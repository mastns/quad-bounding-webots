"""Python port of y_th_model_fpfind3_2.m (the 2007 CSL fixed-point finder used by gtd_controller8).

Ported mechanically from the MATLAB file: every formula is token-for-token the original
(operators spelled the Python way). The functions run in real arithmetic, like MATLAB does
normally; if a step leaves the real domain (sqrt of a negative number, asin out of [-1, 1])
the whole function is re-evaluated in complex arithmetic, as MATLAB would, and the real part is
returned with a warning.
"""
import cmath
import math


def _real(values, where):
    """MATLAB keeps going with complex numbers; Webots motors need reals. Keep the real part."""
    out = []
    for v in values:
        if isinstance(v, complex):
            if abs(v.imag) > 1e-9:
                print('[%s] warning: complex result %r, using its real part' % (where, v))
            v = v.real
        out.append(v)
    return out


def _y_th_model_fpfind3_2_body(F, m, l0, g, k, spl, xddes, h):
    # Program: y_th_model_fpfind3_2.m (version 3, rev 2)
    # created: 26-01-07
    # Reference: Notes p.
    # Mathematica files: stance_duration_spl.nb
    # Description:
    # This program finds a fixed point of the simplified i=md^2 system.
    # The fixed point will have thdapex=0, thapex=0.
    # version 2:
    # In the second version, the constraint gsum=tstbl*xd/(2*l0)
    # is no longer required to hold. This allows fixed points that the previous
    # version did not.
    # version 3:
    # The third version can be called by the simulator.
    # rev 1:
    # uses an analytical method.
    # rev 2:
    # Same as rev1, but incorporates spring preload.
    tst0=F.pi/F.sqrt(2*k/m)
    xd=xddes
    a0 = ((-1/2)*k**(-1)*(2*k*l0**2+(-1)*m*xd**2)**(-1)*(2*g*k* l0**2*m+(-1)*g*m**2*xd**2+
        (-2)*F.sqrt(2)*g*k**(3/2)*l0**2* F.sqrt(m)*tst0*F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)+
        F.sqrt(2)*g* F.sqrt(k)*m**(3/2)*tst0*xd**2*F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)* tst0)+
        4*k**(3/2)*l0**2*F.sqrt(m)*F.sqrt(g*(h+
        (-1)*l0*F.cos((1/2) *l0**(-1)*tst0*xd)))*F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)+
        ( -2)*F.sqrt(k)*m**(3/2)*xd**2*F.sqrt(g*(h+
        (-1)*l0*F.cos((1/2)* l0**(-1)*tst0*xd)))*F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)+
        k**( 3/2)*m**(-1/2)*F.cos((1/2)*l0**(-1)*tst0*xd)*((-4)*F.sqrt(k)* l0**2*F.sqrt(m)*(l0+spl)+
        F.sqrt(2)*tst0*(4*k*l0**2*spl+m*(l0+( -1)*spl)*xd**2)*F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0))+
        (-2)* k**2*l0**2*tst0*xd*F.sin((1/2)*l0**(-1)*tst0*xd)+
        (-2)*k**2* l0*spl*tst0*xd*F.sin((1/2)*l0**(-1)*tst0*xd)+
        2*F.sqrt(2)*k**( 3/2)*l0**2*F.sqrt(m)*xd*F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)* F.sin((1/2)*l0**(-1)*tst0*xd)+
        2*F.sqrt(2)*k**(3/2)*l0*F.sqrt(m)* spl*xd*F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)*F.sin((1/2)*l0**( -1)*tst0*xd)+
        2*F.sqrt(2)*k**(5/2)*l0**2*m**(-1/2)*tst0**2* xd*F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)*F.sin((1/2)*l0**(-1)* tst0*xd)+
        (-2)*F.sqrt(2)*k**(5/2)*l0**3*m**(-1/2)*tst0**2*xd* F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)*F.sin((1/2)*l0**(-1)* tst0*xd)+
        2*F.sqrt(2)*k**(5/2)*l0*m**(-1/2)*spl*tst0**2*xd* F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)*F.sin((1/2)*l0**(-1)* tst0*xd)+
        F.sqrt(2)*k**(3/2)*l0*F.sqrt(m)*tst0**2*xd**3*F.sin( F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)*F.sin((1/2)*l0**(-1)*tst0*xd) +
        (-1)*g*k**(3/2)*l0**2*F.sqrt(m)*tst0*xd*(g*(h+
        (-1)*l0* F.cos((1/2)*l0**(-1)*tst0*xd)))**(-1/2)*F.sin(F.sqrt(2)*F.sqrt(k)* m**(-1/2)*tst0)*F.sin((1/2)*l0**(-1)*tst0*xd)+
        (1/2)*g*F.sqrt(k) *m**(3/2)*tst0*xd**3*(g*(h+
        (-1)*l0*F.cos((1/2)*l0**(-1)* tst0*xd)))**(-1/2)*F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)*F.sin(( 1/2)*l0**(-1)*tst0*xd)+
        (-1/2)*F.cos(F.sqrt(2)*F.sqrt(k)*m**(-1/2) *tst0)*(g*(h+
        (-1)*l0*F.cos((1/2)*l0**(-1)*tst0*xd)))**(-1/2) *(2*g*(2*k*l0**2+(-1)*m*xd**2)*(2*F.sqrt(2)*h*k*tst0+
        m* F.sqrt(g*(h+(-1)*l0*F.cos((1/2)*l0**(-1)*tst0*xd))))+
        (-4)*k* F.cos((1/2)*l0**(-1)*tst0*xd)*(F.sqrt(2)*g*l0*tst0*(2*k* l0**2+(-1)*m*xd**2)+(l0*m*xd**2+
        k*(2*l0**2*spl+l0* tst0**2*xd**2+spl*tst0**2*xd**2))*F.sqrt(g*(h+
        (-1)*l0*F.cos(( 1/2)*l0**(-1)*tst0*xd))))+k*tst0*xd*(F.sqrt(2)*g*tst0*((-2) *k*l0**2+m*xd**2)+
        2*l0*(2*k*(l0+l0**2+spl)+(-1)*m*xd**2) *F.sqrt(g*(h+
        (-1)*l0*F.cos((1/2)*l0**(-1)*tst0*xd))))*F.sin(( 1/2)*l0**(-1)*tst0*xd))))
    a1 = ((1/2)*k**(-1)*(2*k*l0**2+(-1)*m*xd**2)**(-1)*(4*k**2*(l0+
         spl)*tst0*xd**2*F.cos(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)*F.cos(( 1/2)*l0**(-1)*tst0*xd)+
        (-4)*k**2*l0**2*F.cos(F.sqrt(2)*F.sqrt(k) *m**(-1/2)*tst0)*F.sqrt(2*g*h+
        (-2)*g*l0*F.cos((1/2)*l0**(-1) *tst0*xd))+
        2*k*m*xd**2*F.cos(F.sqrt(2)*F.sqrt(k)*m**(-1/2)* tst0)*F.sqrt(2*g*h+
        (-2)*g*l0*F.cos((1/2)*l0**(-1)*tst0*xd))+ F.sqrt(2)*g*F.sqrt(k)*F.sqrt(m)*((-2)*k*l0**2+
        m*xd**2)*F.sin(F.sqrt( 2)*F.sqrt(k)*m**(-1/2)*tst0)+4*F.sqrt(2)*k**(5/2)*l0**2*m**( -1/2)*(l0+
        spl)*F.cos((1/2)*l0**(-1)*tst0*xd)*F.sin(F.sqrt(2)* F.sqrt(k)*m**(-1/2)*tst0)+
        (-1)*F.sqrt(2)*k**(3/2)*F.sqrt(m)*(l0+
         spl)*xd**2*F.cos((1/2)*l0**(-1)*tst0*xd)*F.sin(F.sqrt(2)*F.sqrt(k) *m**(-1/2)*tst0)+
        2*F.sqrt(2)*k**(3/2)*l0*m**(-1/2)*((-2)*k* l0**2+
        m*xd**2)*F.cos((1/2)*l0**(-1)*tst0*xd)*F.sin(F.sqrt(2)* F.sqrt(k)*m**(-1/2)*tst0)+(-2)*k**2*l0*(l0+
        spl)*xd*F.sin((1/2) *l0**(-1)*tst0*xd)+(-2)*k**2*l0*(l0+
        spl)*xd*F.cos(F.sqrt(2)* F.sqrt(k)*m**(-1/2)*tst0)*F.sin((1/2)*l0**(-1)*tst0*xd)+
        k*l0* xd*((-2)*k*l0**2+
        m*xd**2)*F.cos(F.sqrt(2)*F.sqrt(k)*m**(-1/2)* tst0)*F.sin((1/2)*l0**(-1)*tst0*xd)+
        4*g*k**2*l0**2*tst0* xd*F.cos(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)*(2*g*h+
        (-2)*g*l0* F.cos((1/2)*l0**(-1)*tst0*xd))**(-1/2)*F.sin((1/2)*l0**(-1)* tst0*xd)+
        (-2)*g*k*m*tst0*xd**3*F.cos(F.sqrt(2)*F.sqrt(k)*m**( -1/2)*tst0)*(2*g*h+
        (-2)*g*l0*F.cos((1/2)*l0**(-1)*tst0*xd) )**(-1/2)*F.sin((1/2)*l0**(-1)*tst0*xd)+
        4*F.sqrt(2)*k**(5/2)* l0*m**(-1/2)*(l0+
        spl)*tst0*xd*F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2) *tst0)*F.sin((1/2)*l0**(-1)*tst0*xd)+
        2*F.sqrt(2)*k**(3/2)*l0* m**(-1/2)*tst0*xd*((-2)*k*l0**2+
        m*xd**2)*F.sin(F.sqrt(2)*F.sqrt( k)*m**(-1/2)*tst0)*F.sin((1/2)*l0**(-1)*tst0*xd)+
        (-1)*F.sqrt(2) *g*k**(3/2)*l0**2*F.sqrt(m)*xd*(2*g*h+
        (-2)*g*l0*F.cos((1/2) *l0**(-1)*tst0*xd))**(-1/2)*F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)* tst0)*F.sin((1/2)*l0**(-1)*tst0*xd)+
        2**(-1/2)*g*F.sqrt(k)*m**( 3/2)*xd**3*(2*g*h+
        (-2)*g*l0*F.cos((1/2)*l0**(-1)*tst0*xd)) **(-1/2)*F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)*F.sin((1/2)*l0**( -1)*tst0*xd)))
    a2 = ((1/2)*k**(-1)*(2*k*l0**2+(-1)*m*xd**2)**(-1)*((-2)*k**2*( l0+
        spl)*xd**2*F.cos(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)*F.cos((1/2) *l0**(-1)*tst0*xd)+
        (-2)*g*k**2*l0**2*xd*F.cos(F.sqrt(2)*F.sqrt( k)*m**(-1/2)*tst0)*(2*g*h+
        (-2)*g*l0*F.cos((1/2)*l0**(-1)* tst0*xd))**(-1/2)*F.sin((1/2)*l0**(-1)*tst0*xd)+
        g*k*m* xd**3*F.cos(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)*(2*g*h+
        (-2)*g* l0*F.cos((1/2)*l0**(-1)*tst0*xd))**(-1/2)*F.sin((1/2)*l0**(-1)* tst0*xd)+
        (-2)*F.sqrt(2)*k**(5/2)*l0*m**(-1/2)*(l0+
        spl)*xd* F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)*F.sin((1/2)*l0**(-1)* tst0*xd)+
        (-1)*F.sqrt(2)*k**(3/2)*l0*m**(-1/2)*xd*((-2)*k* l0**2+
        m*xd**2)*F.sin(F.sqrt(2)*F.sqrt(k)*m**(-1/2)*tst0)*F.sin((1/2) *l0**(-1)*tst0*xd)))
    tstfp = ((1/4)*a2**(-1)*((-2)*a1+(-1)*l0*xddes*F.sin((1/2)*l0**(-1)* tst0*xddes)+F.sqrt((2*a1+
        l0*xddes*F.sin((1/2)*l0**(-1)*tst0*xddes))**2+ (-8)*a2*(2*a0+(-2)*l0*F.cos((1/2)*l0**(-1)*tst0*xddes)+
        (-1)* l0*tst0*xddes*F.sin((1/2)*l0**(-1)*tst0*xddes)))))
    gsumfp = xddes*tstfp/l0
    out = [gsumfp,tstfp]
    return out


def y_th_model_fpfind3_2(m, l0, g, k, spl, xddes, h):
    """Return [gsumfp, tstfp]: leg sweep angle and stance time of the fixed point."""
    try:
        out = _y_th_model_fpfind3_2_body(math, m, l0, g, k, spl, xddes, h)
    except (ValueError, TypeError):
        out = _y_th_model_fpfind3_2_body(cmath, m, l0, g, k, spl, xddes, h)
    return _real(out, 'y_th_model_fpfind3_2')
