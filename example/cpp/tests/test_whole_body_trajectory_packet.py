"""Packet ownership, provenance refusal and exact ASCII round-trip checks."""
import copy
import hashlib
import json
import pathlib
import sys
import tempfile
import unittest
import mujoco
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'example/cpp/tools/research'))
import export_whole_body_trajectory_packet as exporter


class PacketExporterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        evidence=ROOT/'docs/research/evidence'
        cls.nominal=evidence/'whole_body_cycle_20260908/attempt_0002/result.json'
        cls.feedback=evidence/'whole_body_feedback_20260908/attempt_0001/result.json'
        cls.bundle=exporter.build_packet(cls.nominal,cls.feedback,cls.nominal.with_name('curated_certificate.json'),cls.feedback.with_name('curated_certificate.json'))

    def test_research_scope_and_original_failure(self):
        m=self.bundle[0]
        self.assertFalse(m['production_ready'])
        self.assertFalse(m['execution_ready'])
        self.assertFalse(m['terrain']['observed'])
        self.assertFalse(m['command_authority']['motor_write_authority'])
        self.assertIsNone(m['centroidal_certificate'])
        self.assertFalse(m['coverage']['automatic_loop'])
        self.assertIn('dynamics_balance',m['original_diagnostic_certificates']['nominal']['failflags'])
        self.assertEqual(m['logged_feedback_law_check']['max_command_error_nm'],0.)
        np.testing.assert_array_equal(self.bundle[3],exporter.load(self.nominal)['controls'])

    def test_exact_ascii_roundtrip_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            output=pathlib.Path(directory)/'packet'
            receipt=exporter.write_packet(output,self.bundle)
            lines=(output/'trajectory.packet').read_text().splitlines()
            self.assertEqual(lines[0],'whole-body-trajectory-v1-research')
            expected=hashlib.sha256((output/'manifest.json').read_bytes()).hexdigest()
            self.assertEqual(lines[1],'manifest_sha256 '+expected)
            self.assertEqual(receipt['manifest_sha256'],expected)
            self.assertEqual(receipt['packet_sha256'],exporter.sha(output/'trajectory.packet'))
            samples=[line.split() for line in lines if line.startswith('sample ')]
            self.assertEqual(len(samples),70)
            for k,row in enumerate(samples):
                self.assertEqual(int(row[1]),k)
                self.assertEqual(int(row[2]),self.bundle[0]['coverage']['start_ns_inclusive']+k*2000000)
                expected=np.r_[self.bundle[1][k],self.bundle[2][k],self.bundle[3][k],self.bundle[4][k].ravel()]
                np.testing.assert_array_equal(np.asarray(row[3:],float),expected)
            self.assertEqual(lines[-1],'end')
            with self.assertRaises(FileExistsError):exporter.write_packet(output,self.bundle)

    def test_hash_mismatch_refuses(self):
        with tempfile.TemporaryDirectory() as directory:
            path=pathlib.Path(directory)/'asset.xml';path.write_text('original')
            report={'input_hashes':{'asset.xml':exporter.sha(path)}}
            exporter.verify_inputs(report,pathlib.Path(directory)/'result.json')
            path.write_text('changed')
            with self.assertRaisesRegex(ValueError,'hash mismatch'):
                exporter.verify_inputs(report,pathlib.Path(directory)/'result.json')

    def test_nonfinite_and_shape_refuse(self):
        with self.assertRaises(ValueError):exporter.finite_tree({'nested':[float('nan')]})
        with self.assertRaises(ValueError):exporter.numeric_array([1,2],(3,),'test')

    def test_feedback_mismatch_refuses(self):
        model=mujoco.MjModel.from_xml_path(self.bundle[0]['model']['scene'])
        nominal=exporter.load(self.nominal);feedback=exporter.load(self.feedback)
        feedback['controls'][1][0]+=.1
        with self.assertRaisesRegex(ValueError,'feedback law'):
            exporter.validate_feedback_law(model,nominal,feedback,self.bundle[1],self.bundle[2],self.bundle[4])
        feedback=exporter.load(self.feedback);feedback['initial_integration_state'][1]+=.01
        with self.assertRaisesRegex(ValueError,'initial state mismatch'):
            exporter.validate_feedback_law(model,nominal,feedback,self.bundle[1],self.bundle[2],self.bundle[4])

    def test_initial_observation_binds_qv_and_calendar(self):
        model=mujoco.MjModel.from_xml_path(self.bundle[0]['model']['scene'])
        nominal=exporter.load(self.nominal);initial=mujoco.MjData(model)
        mujoco.mj_setState(model,initial,np.asarray(nominal['initial_integration_state']),nominal['integration_state_spec'])
        tokens=pathlib.Path(self.bundle[0]['observation']['path']).read_text().split()
        self.assertTrue(exporter.validate_observation(model,initial,nominal,tokens)['qpos_qvel_verified'])
        # Source joint order is FR/FL/RR/RL, not MuJoCo qpos order.
        for index in (9,22,34):
            changed=tokens.copy();changed[index]=str(float(changed[index])+.01)
            with self.subTest(index=index), self.assertRaisesRegex(ValueError,'initial q/v mismatch'):
                exporter.validate_observation(model,initial,nominal,changed)
        for index in (3,4,5,6,7):
            changed=tokens.copy();changed[index]=str(float(changed[index])+.01)
            with self.subTest(index=index), self.assertRaisesRegex(ValueError,'calendar/command mismatch'):
                exporter.validate_observation(model,initial,nominal,changed)
    def test_references_are_prestep_not_poststep(self):
        nominal=exporter.load(self.nominal)
        np.testing.assert_array_equal(self.bundle[1][1:],np.asarray([row['qpos'] for row in nominal['rows'][:-1]]))
        np.testing.assert_array_equal(self.bundle[2][1:],np.asarray([row['qvel'] for row in nominal['rows'][:-1]]))
        np.testing.assert_array_equal(self.bundle[-2],nominal['rows'][-1]['qpos'])
        np.testing.assert_array_equal(self.bundle[-1],nominal['rows'][-1]['qvel'])
    def test_feedback_time_shift_and_missing_absolute_marker_refuse(self):
        model=mujoco.MjModel.from_xml_path(self.bundle[0]['model']['scene'])
        nominal=exporter.load(self.nominal)
        for mutation in ('time','absolute'):
            feedback=exporter.load(self.feedback)
            if mutation=='time': feedback['rows'][0]['time']+=.002
            else: feedback['times_are_absolute']=False
            with self.subTest(mutation=mutation), self.assertRaisesRegex(ValueError,'absolute'):
                exporter.validate_feedback_law(model,nominal,feedback,self.bundle[1],self.bundle[2],self.bundle[4])
    def test_certificate_verdict_source_and_coverage_refuse(self):
        original=exporter.load(self.nominal.with_name('curated_certificate.json'))
        with tempfile.TemporaryDirectory() as directory:
            path=pathlib.Path(directory)/'certificate.json'
            for mutation in ('verdict','typed_check','missing_check','source','steps'):
                cert=copy.deepcopy(original)
                if mutation=='verdict': cert['diagnostic_success']=True
                elif mutation=='typed_check': cert['checks']['state_reproduction']=1
                elif mutation=='missing_check': del cert['checks']['dynamics_balance']
                elif mutation=='source': cert['source_sha']='0'*40
                else: cert['steps']+=1
                path.write_text(json.dumps(cert))
                with self.subTest(mutation=mutation), self.assertRaisesRegex(ValueError,'certificate'):
                    exporter.certificate(path,self.nominal)
    def test_certificate_identity_refuses(self):
        with tempfile.TemporaryDirectory() as directory:
            cert=pathlib.Path(directory)/'cert.json'
            cert.write_text(json.dumps({'schema':'whole-body-shooting-certificate-v1','result_sha256':'0'*64}))
            with self.assertRaisesRegex(ValueError,'identity mismatch'):exporter.certificate(cert,self.nominal)


if __name__=='__main__':unittest.main()
