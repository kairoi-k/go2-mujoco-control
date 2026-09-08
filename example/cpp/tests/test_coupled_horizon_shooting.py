"""Independent analytic/LP counterexample proving future-to-earlier coupling."""
import pathlib,sys,unittest
from unittest.mock import patch
import numpy as np
from scipy.optimize import linprog
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'tools/research'))
from coupled_horizon_shooting import solve,HorizonInputError
class CoupledHorizonTests(unittest.TestCase):
    def setUp(self):
        self.A=np.array([[1.,1.],[0.,1.]])
        self.B=np.array([.5,1.])
    def problem(self, target):
        def evaluate(u):
            x=np.zeros(2)
            for stage in u: x=self.A@x+self.B*stage[0]
            return .5*float(np.sum(u*u)),np.array([x[0]-target])
        return evaluate
    def test_future_constraint_changes_earlier_control(self):
        result=solve(self.problem(1.5),np.zeros((2,1)),-1,1)
        self.assertEqual(result['status'],'feasible_evaluated_witness')
        # Analytic Euclidean projection onto 1.5u0+.5u1 >= 1.5 gives(.9,.3).
        np.testing.assert_allclose(result['controls'].ravel(),[.9,.3],atol=2e-7)
        self.assertAlmostEqual(result['cost'],.45,places=7)
        # Independently linear-program exact feasibility; one-step greedy u0=0
        # cannot reach the terminal bound with any bounded final control.
        oracle=linprog([0.,0.],A_ub=[[-1.5,-.5]],b_ub=[-1.5],bounds=[(-1,1)]*2,method='highs')
        greedy=linprog([0.],A_ub=[[-.5]],b_ub=[-1.5],bounds=[(-1,1)],method='highs')
        self.assertTrue(oracle.success)
        self.assertEqual(greedy.status,2)
    def test_prefix_cannot_be_changed_to_escape_infeasibility(self):
        result=solve(self.problem(1.5),np.zeros((2,1)),-1,1,fixed_prefix_steps=1)
        self.assertIsNone(result['controls'])
        self.assertFalse(result['global_infeasibility'])
    def test_feasible_prefix_is_preserved_exactly(self):
        initial=np.array([[.8],[0.]])
        result=solve(self.problem(1.5),initial,-1,1,fixed_prefix_steps=1)
        self.assertEqual(result['controls'][0,0],.8)
        self.assertAlmostEqual(result['controls'][1,0],.6,places=6)
        np.testing.assert_array_equal(initial,[[.8],[0.]])
    def test_unknown_input_propagates(self):
        def unknown(u):raise HorizonInputError('terrain coverage unknown')
        with self.assertRaisesRegex(HorizonInputError,'coverage'):solve(unknown,np.zeros((2,1)),-1,1)
    def test_nonfinite_fails_closed(self):
        result=solve(lambda u:(0.,[np.nan]),np.zeros((2,1)),-1,1)
        self.assertEqual(result['status'],'numerical_failure');self.assertIsNone(result['controls'])
    def test_expired_budget_does_not_publish(self):
        result=solve(self.problem(1.5),np.zeros((2,1)),-1,1,wall_budget_s=1e-12)
        self.assertEqual(result['status'],'wall_budget_exhausted');self.assertIsNone(result['controls'])
    def test_constraint_dimension_change_fails_closed(self):
        calls=[0]
        def changing(u):calls[0]+=1;return float(np.sum(u*u)),np.ones(calls[0])
        result=solve(changing,np.zeros((2,1)),-1,1)
        self.assertEqual(result['status'],'numerical_failure')
    def test_evaluation_finishing_after_deadline_is_rejected(self):
        # The last fresh evaluation begins on time but finishes late.
        clock=[0.]
        calls=[0]
        def evaluation(u):
            calls[0]+=1
            if calls[0]==2:clock[0]=2.
            return 0.,np.ones(1)
        with patch('coupled_horizon_shooting.time.perf_counter',side_effect=lambda:clock[0]):
            result=solve(evaluation,np.zeros((1,1)),-1,1,fixed_prefix_steps=1,wall_budget_s=1.)
        self.assertEqual(result['status'],'wall_budget_exhausted')
        self.assertIsNone(result['controls'])
    def test_all_fixed_is_checked(self):
        result=solve(self.problem(1.5),np.array([[.9],[.3]]),-1,1,fixed_prefix_steps=2)
        self.assertEqual(result['status'],'feasible_evaluated_witness')
if __name__=='__main__':unittest.main()
