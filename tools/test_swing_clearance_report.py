import unittest
from report_swing_clearance import summarize


class SwingSummaryTest(unittest.TestCase):
    def test_censoring_and_same_instant_budget(self):
        def event(delay, complete=True, left=False, body=.001):
            return dict(left_censored=left, complete=complete, liftoff_ms=delay,
                        recontacts=1, measured_peak_m=.02, peak_closure_m=1e-17,
                        at_command_peak={"body_dz_m": body})
        result = summarize([event(10), event(30), event(-1),
                            event(-1, complete=False, body=999),
                            event(1, left=True, body=999)])
        self.assertEqual(result["first_contact_loss_ms_median_observed_only"], 20)
        self.assertEqual(result["completed_without_contact_loss"], 1)
        self.assertEqual(result["censored_without_contact_loss"], 1)
        self.assertEqual(result["left_censored"], 1)
        self.assertEqual(result["complete_with_recontact"], 3)
        self.assertEqual(result["mean_budget_at_command_peak_m"], {"body_dz_m": .001})

    def test_empty(self):
        self.assertIsNone(summarize([])["first_contact_loss_ms_median_observed_only"])
        self.assertEqual(summarize([])["mean_budget_at_command_peak_m"], {})


if __name__ == "__main__":
    unittest.main()
