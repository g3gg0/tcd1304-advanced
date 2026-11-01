import json
import os
from pathlib import Path
import sys
import shutil
import unittest
import uuid
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from build_metadata import collect, write_header


class BuildMetadataTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parents[1] / '.pio' / 'metadata-tests'
        self.project = self.root / uuid.uuid4().hex
        (self.project / 'include').mkdir(parents=True)
        self.addCleanup(self.cleanup_project)
        (self.project / 'include' / 'main.h').write_text('#define CCD_fm 2000000\n')

    def cleanup_project(self):
        # Delete only the unique fixture directory created by this test.
        self.assertEqual(self.project.resolve().parent, self.root.resolve())
        shutil.rmtree(self.project)

    def test_git_build_with_reproducible_utc_timestamp(self):
        responses = {
            ('rev-parse', 'HEAD'): '9376e02864f1fc524f94398520c686b4e46f92af',
            ('status', '--porcelain', '--untracked-files=normal'): ' M src/main.c',
            ('rev-parse', '--abbrev-ref', 'HEAD'): 'master',
            ('describe', '--tags', '--always'): 'v1.1.0-2-g9376e02',
        }
        with patch('build_metadata.git_output', side_effect=lambda project, *args: responses.get(args)), patch.dict(os.environ, {'SOURCE_DATE_EPOCH': '0'}):
            info = collect(self.project, cpu_hz='84000000L', board='nucleo_f401re')
        self.assertEqual(info['built_at'], '1970-01-01T00:00:00Z')
        self.assertEqual(info['commit'], responses[('rev-parse', 'HEAD')])
        self.assertTrue(info['dirty'])
        self.assertEqual(info['cpu_hz'], 84000000)
        self.assertEqual(info['ccd_clock_hz'], 2000000)

    def test_source_archive_reports_missing_git_as_unknown(self):
        with patch('build_metadata.git_output', return_value=None):
            info = collect(self.project)
        self.assertIsNone(info['commit'])
        self.assertIsNone(info['dirty'])
        self.assertTrue(info['built_at'].endswith('Z'))

    def test_json_and_c_escaping_preserve_branch_strings(self):
        info = {'branch': 'feature/"quoted"\\path-ä', 'protocol': 1}
        destination = self.project / 'generated' / 'build_info.generated.h'
        write_header(destination, info)
        literal = destination.read_text().split('#define BUILD_INFO_JSON ', 1)[1].splitlines()[0]
        self.assertEqual(json.loads(json.loads(literal)), info)
        modified = destination.stat().st_mtime_ns
        write_header(destination, info)
        self.assertEqual(destination.stat().st_mtime_ns, modified)

    def test_oversized_payload_rejected_before_emitting_header(self):
        destination = self.project / 'build_info.generated.h'
        with self.assertRaisesRegex(ValueError, '1024'):
            write_header(destination, {'oversized': 'x' * 1025})
        self.assertFalse(destination.exists())


if __name__ == '__main__':
    unittest.main()
