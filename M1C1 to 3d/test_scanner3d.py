from scanner3d import sweep_to_3d, Scanner3D

def test_zero_angle():
    pts=sweep_to_3d([(0,1000),(90,1000),(180,1000),(270,1000)],0,offset=0)
    expected=[(0,1000,0),(1000,0,0),(0,-1000,0),(-1000,0,0)]
    for got,want in zip(pts,expected):
        assert all(abs(a-b)<1e-6 for a,b in zip(got,want)),(got,want)

def test_tilt():
    x,y,z=sweep_to_3d([(0,1000)],90,offset=0)[0]
    assert abs(x)<1e-6 and abs(y)<1e-6 and abs(z-1000)<1e-6

def test_servo_angles():
    class Dummy: pass
    s=Scanner3D(Dummy(),Dummy(),angle_step=2)
    a=s.get_servo_angles()
    assert a[0]==-90 and a[-1]==90 and len(a)==91
    s.set_angle_step(7)
    a=s.get_servo_angles()
    assert a[0]==-90 and a[-1]==90

if __name__=="__main__":
    test_zero_angle(); test_tilt(); test_servo_angles()
    print("All scanner geometry tests passed.")
