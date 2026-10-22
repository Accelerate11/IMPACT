import importlib.util
import sys
from pathlib import Path
import numpy as np
import pytest

SCRIPTS=Path(__file__).parents[1]
sys.path.insert(0,str(SCRIPTS))
spec=importlib.util.spec_from_file_location('joint_evaluator',SCRIPTS/'analyze_joint_livo_replay.py')
e=importlib.util.module_from_spec(spec);spec.loader.exec_module(e)


def geometry():
    prior={'state':{'R':np.eye(3).tolist(),'p':[[.1],[0],[0]]},'clone_R':np.eye(3).tolist(),
           'clone_p':[[0],[0],[0]],'plane':[[0],[0],[2]]}
    protocol={'camera_K':[100,100,100,80],'body_from_optical':np.eye(3).tolist(),
              'optical_origin_body_m':[[0],[0],[0]]}
    return prior,protocol


def test_independent_known_plane_translation_and_height_sensitivity():
    prior,protocol=geometry()
    np.testing.assert_allclose(e.project(prior,[100,80],protocol),[95,80],atol=1e-12)
    np.testing.assert_allclose(e.project(prior,[120,80],protocol),[115,80],atol=1e-12)
    expected={3:[-50,0],4:[0,-50],5:[7.5,0],22:[50,0],27:[2.5,0]}
    for axis,derivative in expected.items():
        d=np.zeros(28);d[axis]=1e-5
        actual=(e.project(prior,[120,80],protocol,d)-e.project(prior,[120,80],protocol,-d))/2e-5
        np.testing.assert_allclose(actual,derivative,atol=1e-8)


@pytest.mark.parametrize('delta',[[.1,.03,-.02],[0,0,0],[1e-8,-1e-8,0]])
def test_right_reset_matches_perturbed_rotation_composition(delta):
    delta=np.array(delta);j=np.zeros((3,3));eps=1e-6
    for axis in range(3):
        d=np.zeros(3);d[axis]=eps
        a=e.exp(-delta)@e.exp(delta+d);b=e.exp(-delta)@e.exp(delta-d)
        difference=(a-b)/(2*eps)
        j[:,axis]=[difference[2,1],difference[0,2],difference[1,0]]
    np.testing.assert_allclose(e.reset(delta),j,atol=1e-9)


def test_covariance_rejects_negative_direction_even_with_positive_diagonal():
    with pytest.raises(ValueError,match='Indefinite'):e.psd([[1,2],[2,1]])


def test_covariance_accepts_exact_clone_semidefinite_nullspace():
    assert e.psd([[1,1],[1,1]])==0


def test_covariance_does_not_silently_symmetrize_invalid_input():
    with pytest.raises(ValueError,match='Asymmetric'):e.psd([[1,.2],[.1,1]])


def test_reconstruction_comparison_detects_changed_gain_or_state():
    with pytest.raises(ValueError,match='gain mismatch'):e.close(np.eye(2)*2,np.eye(2),1e-10,'gain')
