import tempfile
import unittest
from pathlib import Path

from controller.dashboard import Diagnostics


class DebuggerFeedbackTests(unittest.TestCase):
    def test_feedback_is_persisted_and_returned_in_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            diagnostics = Diagnostics(Path(directory) / 'operations.jsonl')
            feedback = diagnostics.add_feedback({
                'kind': 'navigation',
                'comment': 'The robot should have stopped before the obstacle.',
                'expected_action': 'Stop and request review',
                'decision_id': 42,
            })
            self.assertEqual(feedback['decision_id'], 42)
            self.assertEqual(diagnostics.snapshot(compact=True)['feedback'][0]['kind'], 'navigation')

    def test_feedback_rejects_empty_comments(self):
        with tempfile.TemporaryDirectory() as directory:
            diagnostics = Diagnostics(Path(directory) / 'operations.jsonl')
            with self.assertRaises(ValueError):
                diagnostics.add_feedback({'comment': ' '})


if __name__ == '__main__':
    unittest.main()
