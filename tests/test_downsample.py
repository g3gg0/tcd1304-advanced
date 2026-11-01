"""Run the actual firmware compactor/encoder natively and check its wire bytes."""
import ctypes
import math
import os
from pathlib import Path
import shutil
import subprocess
import unittest
import uuid

ROOT = Path(__file__).resolve().parents[1]


class DownsampleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = ROOT / '.pio' / 'packing-tests' / uuid.uuid4().hex
        cls.fixture.mkdir(parents=True)
        source = cls.fixture / 'native.c'
        source.write_text(''.join(f'#include "{(ROOT / "src" / file).as_posix()}"\n' for file in ['downsample.c', 'acquisition.c']), encoding='utf-8')
        if os.name == 'nt':
            vcvars = Path('C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Auxiliary/Build/vcvars64.bat')
            if not vcvars.exists():
                raise unittest.SkipTest('Native C checks require MSVC or a POSIX C compiler')
            cls.library = cls.fixture / 'packing.dll'
            batch = cls.fixture / 'compile.cmd'
            batch.write_text(f'@echo off\ncall "{vcvars}" >nul\ncl /nologo /LD /O2 /TC /I"{ROOT / "include"}" /Fo"{cls.fixture / "packing.obj"}" "{source}" /link /OUT:"{cls.library}" /EXPORT:downsample_frame /EXPORT:pack_samples /EXPORT:acquisition_prepare /EXPORT:acquisition_finished\n', encoding='utf-8')
            subprocess.run(['cmd', '/d', '/c', str(batch)], cwd=cls.fixture, check=True, capture_output=True)
        else:
            cls.library = cls.fixture / 'packing.so'
            subprocess.run([os.environ.get('CC', 'cc'), '-shared', '-fPIC', '-O2', '-I', str(ROOT / 'include'), str(source), '-o', str(cls.library)], check=True)
        cls.lib = ctypes.CDLL(str(cls.library))
        cls.lib.downsample_frame.argtypes = [ctypes.POINTER(ctypes.c_uint16), ctypes.c_uint16, ctypes.c_uint16, ctypes.POINTER(ctypes.c_uint8)]
        cls.lib.downsample_frame.restype = ctypes.c_uint16
        cls.lib.pack_samples.argtypes = [ctypes.POINTER(ctypes.c_uint16), ctypes.c_uint16, ctypes.c_uint8]
        cls.lib.pack_samples.restype = ctypes.c_uint16

    @classmethod
    def tearDownClass(cls):
        if os.name == 'nt':
            import _ctypes
            _ctypes.FreeLibrary(cls.lib._handle)
        del cls.lib
        resolved = cls.fixture.resolve()
        assert resolved.is_relative_to((ROOT / '.pio' / 'packing-tests').resolve())
        shutil.rmtree(resolved)

    def wire(self, values, stride, encoding):
        samples = (ctypes.c_uint16 * len(values))(*values)
        header = (ctypes.c_uint8 * 16)()
        count = self.lib.downsample_frame(samples, len(values), stride, header)
        length = self.lib.pack_samples(samples, count, encoding)
        return count, bytes(header), ctypes.string_at(samples, length)

    def test_all_strides_and_encodings_match_independent_wire_encoder(self):
        values = [(i * 131 + 17) % 4096 for i in range(3694)]
        for stride in range(1, 65):
            for encoding, bits in [(0, 16), (1, 10), (2, 12), (3, 8)]:
                with self.subTest(stride=stride, encoding=encoding):
                    selected = values[::stride]
                    expected = bytearray(math.ceil(len(selected) * bits / 8))
                    offset = 0
                    for value in selected:
                        code = int(math.floor(math.log1p(value) / math.log(4096) * ((1 << bits) - 1) + .5)) if encoding in [1, 3] else value
                        for bit in range(bits):
                            if code & (1 << bit):
                                expected[offset // 8] |= 1 << (offset % 8)
                            offset += 1
                    count, header, payload = self.wire(values, stride, encoding)
                    self.assertEqual(count, len(selected))
                    self.assertEqual(header[:4], b'DSF1')
                    self.assertEqual(int.from_bytes(header[4:6], 'big'), stride)
                    self.assertEqual(int.from_bytes(header[6:8], 'big'), count)
                    self.assertEqual(payload, expected)

    def test_fast_clock_requires_completed_fast_frame_and_identical_timing(self):
        class Clock(ctypes.Structure):
            _fields_ = [('sh', ctypes.c_uint32), ('icg', ctypes.c_uint32), ('ready', ctypes.c_uint8)]
        prepare = self.lib.acquisition_prepare
        prepare.argtypes = [ctypes.POINTER(Clock), ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint8]
        prepare.restype = ctypes.c_uint8
        finish = self.lib.acquisition_finished
        finish.argtypes = [ctypes.POINTER(Clock), ctypes.c_uint8]
        clock = Clock()
        self.assertEqual(prepare(ctypes.byref(clock), 20, 14800, 1), 0)  # Initial flush.
        finish(ctypes.byref(clock), 1)
        for _ in range(1000):
            self.assertEqual(prepare(ctypes.byref(clock), 20, 14800, 1), 1)
            finish(ctypes.byref(clock), 1)
        self.assertEqual(prepare(ctypes.byref(clock), 40, 14800, 1), 0)  # Changed SH.
        finish(ctypes.byref(clock), 1)
        self.assertEqual(prepare(ctypes.byref(clock), 40, 16000, 1), 0)  # Changed ICG.
        finish(ctypes.byref(clock), 1)
        self.assertEqual(prepare(ctypes.byref(clock), 40, 16000, 0), 0)  # Calibration/trigger/legacy.
        finish(ctypes.byref(clock), 0)
        self.assertEqual(prepare(ctypes.byref(clock), 40, 16000, 1), 0)  # Clocks no longer valid.
        self.assertEqual(prepare(ctypes.byref(clock), 40, 16000, 1), 0)  # No completion, no resume.

    def test_health_retains_clipping_and_invalid_samples_off_grid(self):
        values = [100] * 3694
        values[1], values[2], values[3] = 4095, 65535, 0
        count, header, payload = self.wire(values, 58, 3)
        self.assertEqual(count, 64)
        self.assertEqual([int.from_bytes(header[i:i+2], 'big') for i in range(8, 16, 2)], [0, 65535, 1, 1])
        self.assertEqual(len(set(payload)), 1)

    def test_log_lookup_tables_cover_every_adc_code_and_preserve_endpoints(self):
        for encoding, bits in [(1, 10), (3, 8)]:
            _, _, payload = self.wire(list(range(4096)), 1, encoding)
            previous = -1
            for i in range(4096):
                code = sum(((payload[(i*bits+b)//8] >> ((i*bits+b)%8)) & 1) << b for b in range(bits))
                expected = math.floor(math.log1p(i) / math.log(4096) * ((1 << bits)-1) + .5)
                self.assertEqual(code, expected)
                self.assertGreaterEqual(code, previous)
                previous = code
            self.assertEqual(previous, (1 << bits)-1)
