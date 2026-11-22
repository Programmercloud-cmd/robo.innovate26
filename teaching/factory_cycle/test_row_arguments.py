"""All-column expansion is shared by each public launcher; no hardware."""
from contextlib import redirect_stderr
import io
import unittest

from run_rows import parse_args


class RowArgumentTests(unittest.TestCase):
    def test_all_expands_all_eighteen_once_in_order_for_every_setup(self):
        expected=[f'{row}_{col}' for row in range(6) for col in range(3)]
        for setup in ('tabletop','conveyor','conveyor_moving'):
            args=parse_args(['--setup',setup,'--execute-task','--all'])
            self.assertEqual(args.columns,expected)
            self.assertEqual(len(set(args.columns)),18)
            self.assertTrue(args.execute_task)
            self.assertTrue(args.all_columns)

    def test_explicit_selection_keeps_operator_order(self):
        args=parse_args(['--setup','conveyor','--execute-task','5_2','0_0'])
        self.assertEqual(args.columns,['5_2','0_0']);self.assertFalse(args.all_columns)

    def test_empty_ambiguous_duplicate_or_invalid_selection_stops_at_parsing(self):
        for selection in ([],['--all','5_0'],['5_0','5_0'],['6_0']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as result:
                parse_args(['--setup','conveyor']+selection)
            self.assertEqual(result.exception.code,2)


if __name__=='__main__':unittest.main()
