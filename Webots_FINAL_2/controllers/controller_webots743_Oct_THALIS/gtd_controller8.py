"""Python port of gtd_controller8.m (N. Cherouvim, CSL-EP 2007; adapted by A. Mastrogeorgiou).

Ported mechanically from the MATLAB file: every formula is token-for-token the original
(operators spelled the Python way). Real arithmetic first; if a step leaves the real domain the
function is re-evaluated in complex arithmetic, as MATLAB would, and the real part is returned
with a warning.
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

from y_th_model_fpfind3_2 import _y_th_model_fpfind3_2_body


def _gtd_controller8_body(F, m, ib, mt, mul, ml, il, l1, d, l0, g, k, b, spl, psia,
                          xddes, h, xdlo, ylo, ydlo, thlo, thdlo, gamblo, gamflo, gambdlo, gamfdlo):
    # Program: <gtd_controller8.m>  (version 8, rev )
    # created: 20-01-07 - ndcherouvim @CSL-EP
    # %
    # Description:
    # This is the controller file for some of the gtd_control3_*.m family.
    # (at least gtd_control3_10.m)
    # version 2:
    # This is based on the controller from the WM2D uphill series,
    # gtd_control2_5_contr_uv16.m
    # version 3:
    # Use new pfp data finder (y_th_model_fpfind4). Use modified xddes in most
    # instances.
    # version 4:
    # Using real xd in call of gdif finder. Change gambdm to more correct value
    # of current state. Special calculation of slopeloss at *current* state for
    # downhill case ONLY.
    # version 5:
    # Based on v4, but with extra changes made in up to wm3d_uv37.m. Then
    # further changes are made for the new apex height control using direct
    # gsum method. Also, it is made a function. The gdif finder is now
    # y_th_model_gtdfind8.m. Now applicable for deviations from i=md^2.
    # version 6:
    # No longer uses numerical solving to find gdif. Now uses analysis from
    # pitch_control_gdif_ds_only.nb.
    # version 7:
    # General improvements. Better prediction of body pitching state (after the
    # positioning of the legs in flight), for computation of the necessary
    # gdif.
    # version 8:
    # This version has support for spring preload in the legs.
    # ----------------------- 1. THANASIS -------------------------------------&
    # ------- constant parameters -------------
    # m: robot body mass
    # ib: robot body inertia
    # mt: robot total mass
    # mul: upper leg mass
    # ml: total leg mass
    # il: leg inertia
    # l1: distance from leg CoM to hip joint
    # d: hip joint to body CoM distance
    # l0: leg uncompressed length
    # g: gravity accel
    # k: spring constant
    # b: dumping constant
    # spl: spring preload = 0
    # psia: ground inclination =0
    # xddes: desired forward speed =1.0 m/s
    # h: desired apex height = 0.32m
    # ------ liftoff parameters -----------
    # xdlo: forward speed body CoM = Acc me oloklirosi CoM
    # ylo: vertical position body CoM = GPS CoM
    # ydlo: vertical velocity body CoM = Acc me oloklirosi CoM
    # thlo: pitch angle body = oloklirosi Gyro (pitch)
    # thdlo: pitch rate body = Gyro (pitch)
    # gamblo: back leg angle = encoder
    # gamflo: front leg angle = encoder
    # gambdlo: back leg velocity
    # gamfdlo: front leg velocity
    # ----------------------- 1. THANASIS ----------------------------------------&
    # Parameters:
    # k=2*k;
    gy = F.cos(psia)*g
    gx = g*F.sin(psia)
    # Quantities for pfp, afp.
    yth_fp_data = _y_th_model_fpfind3_2_body(F, m+2*mul,l0,gy,k,spl,xddes,h)
    gsumpfp = yth_fp_data[0]
    tstpfp = yth_fp_data[1]
    gdiffp = 0
    xddesmod = xddes+2*F.sqrt(2*(h-l0*F.cos(gsumpfp/2))*gy)*gx/gy
    xdafp = (xddes+xddesmod)/2
    # Current apex, td quantities.
    yapex = ylo+0.5*ydlo**2/gy
    xdtd = (xdlo+(ml*l1/mt)*(gambdlo*F.cos(gamblo)+gamfdlo*F.cos(gamflo)) -gx*(ydlo/gy+
        F.sqrt(2*gy*(yapex-l0*F.cos(xdafp*tstpfp/(2*l0))))/gy))
    xdrl=(xdtd+xddesmod)/2
    # ---------------------- Leg touchdown angles -----------------------------
    # For gf* variables, see mathematica file: gsum_direct_height_control.nb.
    # afp analysis
    # ------------------------- gsumafp calculation ---------------------------%
    gff1 = ((-2)*F.exp((-1)*b*m**(-1)*tstpfp)*k*l0**2*((-1)*b**2+2*k*m)**( -1/2)*(l0+spl)*(4*k**2*l0**4+
        4*l0**2*(b**2+(-1)*k*m)* xdafp**2+m**2*xdafp**4)**(-1)*(F.sqrt((-1)*b**2+2*k*m)*(2*k*l0**2+
         (-1)*m*xdafp**2)*F.cos(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+
        F.exp(b* m**(-1)*tstpfp)*F.sqrt((-1)*b**2+2*k*m)*((-2)*k*l0**2+
        m*xdafp**2)* F.cos(l0**(-1)*tstpfp*xdafp)+b*(2*k*l0**2+m*xdafp**2)*F.sin(m**(-1)*F.sqrt( (-1)*b**2+
        2*k*m)*tstpfp)+(-2)*b*F.exp(b*m**(-1)*tstpfp)*l0*F.sqrt((-1) *b**2+
        2*k*m)*xdafp*F.sin(l0**(-1)*tstpfp*xdafp)))
    gff2 = (2*F.exp((-1)*b*m**(-1)*tstpfp)*k*l0*((-1)*b**2+2*k*m)**(-1/2)* (l0+spl)*(4*k**2*l0**4+
        4*l0**2*(b**2+(-1)*k*m)*xdafp**2+ m**2*xdafp**4)**(-1)*(2*b*l0**2*F.sqrt((-1)*b**2+
        2*k*m)*xdafp* F.cos(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+
        (-2)*b*F.exp(b*m**(-1) *tstpfp)*l0**2*F.sqrt((-1)*b**2+2*k*m)*xdafp*F.cos(l0**(-1)*tstpfp*xdafp)+
         xdafp*(2*l0**2*(b**2+(-1)*k*m)+m**2*xdafp**2)*F.sin(m**(-1)*F.sqrt(( -1)*b**2+2*k*m)*tstpfp)+
        2*F.exp(b*m**(-1)*tstpfp)*k*l0**3*F.sqrt((-1) *b**2+2*k*m)*F.sin(l0**(-1)*tstpfp*xdafp)+
        (-1)*F.exp(b*m**(-1)*tstpfp)* l0*m*F.sqrt((-1)*b**2+2*k*m)*xdafp**2*F.sin(l0**(-1)*tstpfp*xdafp)))
    gfg1 = (2*F.exp((-1)*b*m**(-1)*tstpfp)*k*l0*((-1)*b**2+2*k*m)**( -1/2)*(l0+spl)*(4*k**2*l0**4+
        4*l0**2*(b**2+(-1)*k*m)* xdafp**2+m**2*xdafp**4)**(-1)*((-2)*b*l0*F.sqrt((-1)*b**2+
        2*k*m)* xdafp**2*F.cos(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+
        2*b*F.exp( b*m**(-1)*tstpfp)*l0*F.sqrt((-1)*b**2+2*k*m)*xdafp**2*F.cos( l0**(-1)*tstpfp*xdafp)+
        2*l0*(2*k**2*l0**2+(b**2+(-1)*k*m)* xdafp**2)*F.sin(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+
        (-2)*F.exp( b*m**(-1)*tstpfp)*k*l0**2*F.sqrt((-1)*b**2+2*k*m)*xdafp*F.sin( l0**(-1)*tstpfp*xdafp)+
        F.exp(b*m**(-1)*tstpfp)*m*F.sqrt((-1)*b**2+ 2*k*m)*xdafp**3*F.sin(l0**(-1)*tstpfp*xdafp)))
    gfg2 = ((-2)*F.exp((-1)*b*m**(-1)*tstpfp)*k*l0*((-1)*b**2+2*k*m) **(-1/2)*(l0+spl)*xdafp*(4*k**2*l0**4+
        4*l0**2*(b**2+(-1)*k* m)*xdafp**2+m**2*xdafp**4)**(-1)*(F.sqrt((-1)*b**2+2*k*m)*(2*k* l0**2+
        (-1)*m*xdafp**2)*F.cos(m**(-1)*F.sqrt((-1)*b**2+2*k*m)* tstpfp)+
        F.exp(b*m**(-1)*tstpfp)*F.sqrt((-1)*b**2+2*k*m)*((-2)* k*l0**2+
        m*xdafp**2)*F.cos(l0**(-1)*tstpfp*xdafp)+b*(2*k*l0**2+m* xdafp**2)*F.sin(m**(-1)*F.sqrt((-1)*b**2+
        2*k*m)*tstpfp)+(-2)*b* F.exp(b*m**(-1)*tstpfp)*l0*F.sqrt((-1)*b**2+
        2*k*m)*xdafp*F.sin( l0**(-1)*tstpfp*xdafp)))
    eq0 = ((1/2)*gff2*gsumpfp+gff1*(1+(-1/8)*gsumpfp**2)+(-1)*h+
        (1/2)* F.exp((-1)*b*m**(-1)*tstpfp)*k**(-1)*((-1)*b**2+
        2*k*m)**(-1/2)*( (-1)*F.exp(b*m**(-1)*tstpfp)*gy*m*F.sqrt((-1)*b**2+2*k*m)+(2*(1+
        ( -1/8)*gsumpfp**2)*k*l0+gy*m)*F.sqrt((-1)*b**2+2*k*m)*F.cos( m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+
        (2*b*(1+(-1/8)* gsumpfp**2)*k*l0+b*gy*m+(-2)*F.sqrt(2)*F.sqrt(gy)*k*F.sqrt(h+( -1)*(1+
        (-1/8)*gsumpfp**2)*l0)*m)*F.sin(m**(-1)*F.sqrt((-1)* b**2+2*k*m)*tstpfp))+
        (1/2)*gy**(-1)*((1/2)*gfg2*gsumpfp+gfg1*( 1+(-1/8)*gsumpfp**2)+
        F.exp((-1)*b*m**(-1)*tstpfp)*((-1)*b**2+ 2*k*m)**(-1/2)*((-1)*F.sqrt(2)*F.sqrt(gy)*F.sqrt(h+(-1)*(1+
        (-1/8) *gsumpfp**2)*l0)*F.sqrt((-1)*b**2+2*k*m)*F.cos(m**(-1)*F.sqrt(( -1)*b**2+2*k*m)*tstpfp)+
        (-1)*(2*(1+(-1/8)*gsumpfp**2)*k* l0+(-1)*F.sqrt(2)*b*F.sqrt(gy)*F.sqrt(h+(-1)*(1+
        (-1/8)*gsumpfp**2) *l0)+gy*m)*F.sin(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)))**2)
    eqd0 = ((1/4)*(2*gff2+(-1)*gff1*gsumpfp+F.exp((-1)*b*m**(-1)*tstpfp)* gsumpfp*l0*((-1)*b**2+
        2*k*m)**(-1/2)*((-1)*F.sqrt((-1)*b**2+ 2*k*m)*F.cos(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+((-1)*b+
        (-2) *F.sqrt(gy)*(8*h+((-8)+gsumpfp**2)*l0)**(-1/2)*m)*F.sin(m**(-1) *F.sqrt((-1)*b**2+
        2*k*m)*tstpfp))+(1/8)*F.exp((-2)*b*m**(-1)* tstpfp)*gy**(-1)*(8*h+((-8)+gsumpfp**2)*l0)**(-1/2)*(b**2+
        ( -2)*k*m)**(-1)*(F.exp(b*m**(-1)*tstpfp)*(2*gfg2+(-1)*gfg1* gsumpfp)*F.sqrt(8*h+((-8)+
        gsumpfp**2)*l0)*F.sqrt((-1)*b**2+2*k* m)+(-2)*gsumpfp*F.sqrt(gy)*l0*F.sqrt((-1)*b**2+
        2*k*m)*F.cos(m**( -1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+2*gsumpfp*l0*(b*F.sqrt( gy)+k*F.sqrt(8*h+((-8)+
        gsumpfp**2)*l0))*F.sin(m**(-1)*F.sqrt((-1)* b**2+
        2*k*m)*tstpfp))*(F.exp(b*m**(-1)*tstpfp)*((-4)*gfg2* gsumpfp+gfg1*((-8)+gsumpfp**2))*F.sqrt((-1)*b**2+
        2*k*m)+4* F.sqrt(gy)*F.sqrt(8*h+((-8)+gsumpfp**2)*l0)*F.sqrt((-1)*b**2+
        2*k* m)*F.cos(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+(-2)*(((-8)+ gsumpfp**2)*k*l0+
        2*b*F.sqrt(gy)*F.sqrt(8*h+((-8)+gsumpfp**2)* l0)+(-4)*gy*m)*F.sin(m**(-1)*F.sqrt((-1)*b**2+
        2*k*m)*tstpfp))))
    eqdd0 = ((1/4)*((-1)*gff1+2*F.exp((-1)*b*m**(-1)*tstpfp)*k**(-1)*((-1)* b**2+
        2*k*m)**(-1/2)*((-1/2)*k*l0*F.sqrt((-1)*b**2+2*k*m)* F.cos(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+
        (-1/2)*k*l0*(8*h+(( -8)+gsumpfp**2)*l0)**(-3/2)*(b*(8*h+((-8)+gsumpfp**2)*l0)**( 3/2)+
        16*F.sqrt(gy)*(h+(-1)*l0)*m)*F.sin(m**(-1)*F.sqrt((-1)*b**2+ 2*k*m)*tstpfp))+
        (1/8)*F.exp((-2)*b*m**(-1)*tstpfp)*gy**(-1)*(8* h+((-8)+gsumpfp**2)*l0)**(-3/2)*(b**2+
        (-2)*k*m)**(-1)*((-1)* F.exp(b*m**(-1)*tstpfp)*gfg1*(8*h+((-8)+
        gsumpfp**2)*l0)**(3/2) *F.sqrt((-1)*b**2+2*k*m)+(-16)*F.sqrt(gy)*(h+(-1)*l0)*l0* F.sqrt((-1)*b**2+
        2*k*m)*F.cos(m**(-1)*F.sqrt((-1)*b**2+2*k*m)* tstpfp)+2*l0*(8*b*F.sqrt(gy)*(h+(-1)*l0)+k*(8*h+((-8)+
         gsumpfp**2)*l0)**(3/2))*F.sin(m**(-1)*F.sqrt((-1)*b**2+
        2*k*m)* tstpfp))*(F.exp(b*m**(-1)*tstpfp)*((-4)*gfg2*gsumpfp+gfg1*(( -8)+
        gsumpfp**2))*F.sqrt((-1)*b**2+2*k*m)+4*F.sqrt(gy)*F.sqrt(8*h+ ((-8)+gsumpfp**2)*l0)*F.sqrt((-1)*b**2+
        2*k*m)*F.cos(m**(-1)* F.sqrt((-1)*b**2+2*k*m)*tstpfp)+(-2)*(((-8)+gsumpfp**2)*k*l0+
         2*b*F.sqrt(gy)*F.sqrt(8*h+((-8)+gsumpfp**2)*l0)+(-4)*gy*m)* F.sin(m**(-1)*F.sqrt((-1)*b**2+
        2*k*m)*tstpfp))+4*gy**(-1)*(( 1/2)*gfg2+(-1/4)*gfg1*gsumpfp+
        (1/2)*F.exp((-1)*b*m**(-1)* tstpfp)*gsumpfp*l0*(8*h+((-8)+gsumpfp**2)*l0)**(-1/2)*((-1) *b**2+
        2*k*m)**(-1/2)*((-1)*F.sqrt(gy)*F.sqrt((-1)*b**2+2*k*m) *F.cos(m**(-1)*F.sqrt((-1)*b**2+
        2*k*m)*tstpfp)+(b*F.sqrt(gy)+k* F.sqrt(8*h+((-8)+gsumpfp**2)*l0))*F.sin(m**(-1)*F.sqrt((-1)*b**2+
         2*k*m)*tstpfp)))**2))
    gsumafp = eqdd0**(-1)*((-1)*eqd0+F.sqrt(eqd0**2+(-2)*eq0*eqdd0)+eqdd0* gsumpfp)
    # -------------------------------------------------------------------------%
    dxddes = 0
    gambdm = -xdafp/l0
    # -------------------------- gsum calculation -----------------------------%
    # for transient:
    # what could change here is whether we use xdrl or xdafp. Now xdrl is used.
    gff1 = ((-2)*F.exp((-1)*b*m**(-1)*tstpfp)*k*l0**2*((-1)*b**2+2*k*m)**( -1/2)*(l0+spl)*(4*k**2*l0**4+
        4*l0**2*(b**2+(-1)*k*m)* xdrl**2+m**2*xdrl**4)**(-1)*(F.sqrt((-1)*b**2+2*k*m)*(2*k*l0**2+
         (-1)*m*xdrl**2)*F.cos(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+
        F.exp(b* m**(-1)*tstpfp)*F.sqrt((-1)*b**2+2*k*m)*((-2)*k*l0**2+
        m*xdrl**2)* F.cos(l0**(-1)*tstpfp*xdrl)+b*(2*k*l0**2+m*xdrl**2)*F.sin(m**(-1)*F.sqrt( (-1)*b**2+
        2*k*m)*tstpfp)+(-2)*b*F.exp(b*m**(-1)*tstpfp)*l0*F.sqrt((-1) *b**2+
        2*k*m)*xdrl*F.sin(l0**(-1)*tstpfp*xdrl)))
    gff2 = (2*F.exp((-1)*b*m**(-1)*tstpfp)*k*l0*((-1)*b**2+2*k*m)**(-1/2)* (l0+spl)*(4*k**2*l0**4+
        4*l0**2*(b**2+(-1)*k*m)*xdrl**2+ m**2*xdrl**4)**(-1)*(2*b*l0**2*F.sqrt((-1)*b**2+
        2*k*m)*xdrl* F.cos(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+
        (-2)*b*F.exp(b*m**(-1) *tstpfp)*l0**2*F.sqrt((-1)*b**2+2*k*m)*xdrl*F.cos(l0**(-1)*tstpfp*xdrl)+
         xdrl*(2*l0**2*(b**2+(-1)*k*m)+m**2*xdrl**2)*F.sin(m**(-1)*F.sqrt(( -1)*b**2+2*k*m)*tstpfp)+
        2*F.exp(b*m**(-1)*tstpfp)*k*l0**3*F.sqrt((-1) *b**2+2*k*m)*F.sin(l0**(-1)*tstpfp*xdrl)+
        (-1)*F.exp(b*m**(-1)*tstpfp)* l0*m*F.sqrt((-1)*b**2+2*k*m)*xdrl**2*F.sin(l0**(-1)*tstpfp*xdrl)))
    gfg1 = (2*F.exp((-1)*b*m**(-1)*tstpfp)*k*l0*((-1)*b**2+2*k*m)**( -1/2)*(l0+spl)*(4*k**2*l0**4+
        4*l0**2*(b**2+(-1)*k*m)* xdrl**2+m**2*xdrl**4)**(-1)*((-2)*b*l0*F.sqrt((-1)*b**2+
        2*k*m)* xdrl**2*F.cos(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+
        2*b*F.exp( b*m**(-1)*tstpfp)*l0*F.sqrt((-1)*b**2+2*k*m)*xdrl**2*F.cos( l0**(-1)*tstpfp*xdrl)+
        2*l0*(2*k**2*l0**2+(b**2+(-1)*k*m)* xdrl**2)*F.sin(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+
        (-2)*F.exp( b*m**(-1)*tstpfp)*k*l0**2*F.sqrt((-1)*b**2+2*k*m)*xdrl*F.sin( l0**(-1)*tstpfp*xdrl)+
        F.exp(b*m**(-1)*tstpfp)*m*F.sqrt((-1)*b**2+ 2*k*m)*xdrl**3*F.sin(l0**(-1)*tstpfp*xdrl)))
    gfg2 = ((-2)*F.exp((-1)*b*m**(-1)*tstpfp)*k*l0*((-1)*b**2+2*k*m) **(-1/2)*(l0+spl)*xdrl*(4*k**2*l0**4+
        4*l0**2*(b**2+(-1)*k* m)*xdrl**2+m**2*xdrl**4)**(-1)*(F.sqrt((-1)*b**2+2*k*m)*(2*k* l0**2+
        (-1)*m*xdrl**2)*F.cos(m**(-1)*F.sqrt((-1)*b**2+2*k*m)* tstpfp)+
        F.exp(b*m**(-1)*tstpfp)*F.sqrt((-1)*b**2+2*k*m)*((-2)* k*l0**2+m*xdrl**2)*F.cos(l0**(-1)*tstpfp*xdrl)+
        b*(2*k*l0**2+m* xdrl**2)*F.sin(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+
        (-2)*b* F.exp(b*m**(-1)*tstpfp)*l0*F.sqrt((-1)*b**2+2*k*m)*xdrl*F.sin( l0**(-1)*tstpfp*xdrl)))
    eq0 = ((-1)*h+(1/2)*(gff2*gsumafp+(-1/4)*gff1*((-8)+gsumafp**2)+
         F.exp((-1)*b*m**(-1)*tstpfp)*k**(-1)*((-1)*b**2+
        2*k*m)**(-1/2)*( (-1)*F.exp(b*m**(-1)*tstpfp)*gy*m*F.sqrt((-1)*b**2+2*k*m)+((-1/4) *((-8)+
        gsumafp**2)*k*l0+gy*m)*F.sqrt((-1)*b**2+2*k*m)* F.cos(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+
        ((-1/4)*b*((-8)+ gsumafp**2)*k*l0+b*gy*m+(-1)*F.sqrt(gy)*k*m*F.sqrt(((-8)+ gsumafp**2)*l0+
        8*yapex))*F.sin(m**(-1)*F.sqrt((-1)*b**2+2*k* m)*tstpfp))+gy**(-1)*(gfg1+(1/2)*gfg2*gsumafp+
        (-1/8)*gfg1* gsumafp**2+(1/4)*F.exp((-1)*b*m**(-1)*tstpfp)*((-1)*b**2+
        2* k*m)**(-1/2)*((-2)*F.sqrt(gy)*F.sqrt((-1)*b**2+2*k*m)*F.sqrt((( -8)+gsumafp**2)*l0+
        8*yapex)*F.cos(m**(-1)*F.sqrt((-1)*b**2+2* k*m)*tstpfp)+(((-8)+gsumafp**2)*k*l0+(-4)*gy*m+
        2*b*F.sqrt( gy)*F.sqrt(((-8)+gsumafp**2)*l0+8*yapex))*F.sin(m**(-1)*F.sqrt(( -1)*b**2+
        2*k*m)*tstpfp)))**2))
    eqd0 = ((1/2)*(gff2+(-1/2)*gff1*gsumafp+F.exp((-1)*b*m**(-1)*tstpfp)* k**(-1)*((-1)*b**2+
        2*k*m)**(-1/2)*((-1/2)*gsumafp*k*l0* F.sqrt((-1)*b**2+2*k*m)*F.cos(m**(-1)*F.sqrt((-1)*b**2+
        2*k*m)* tstpfp)+((-1/2)*b*gsumafp*k*l0+(-1)*gsumafp*F.sqrt(gy)*k*l0* m*(((-8)+gsumafp**2)*l0+
        8*yapex)**(-1/2))*F.sin(m**(-1)*F.sqrt( (-1)*b**2+2*k*m)*tstpfp))+(1/2)*gy**(-1)*(gfg1+
        (1/2)*gfg2* gsumafp+(-1/8)*gfg1*gsumafp**2+(1/4)*F.exp((-1)*b*m**(-1)* tstpfp)*((-1)*b**2+
        2*k*m)**(-1/2)*((-2)*F.sqrt(gy)*F.sqrt((-1) *b**2+2*k*m)*F.sqrt(((-8)+gsumafp**2)*l0+
        8*yapex)*F.cos(m**( -1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+(((-8)+gsumafp**2)*k* l0+(-4)*gy*m+
        2*b*F.sqrt(gy)*F.sqrt(((-8)+gsumafp**2)*l0+8* yapex))*F.sin(m**(-1)*F.sqrt((-1)*b**2+
        2*k*m)*tstpfp)))*(2* gfg2+(-1)*gfg1*gsumafp+(-2)*F.exp((-1)*b*m**(-1)*tstpfp)* gsumafp*l0*((-1)*b**2+
        2*k*m)**(-1/2)*(((-8)+gsumafp**2) *l0+8*yapex)**(-1/2)*(F.sqrt(gy)*F.sqrt((-1)*b**2+
        2*k*m)*F.cos( m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+(-1)*(b*F.sqrt(gy)+k* F.sqrt(((-8)+
        gsumafp**2)*l0+8*yapex))*F.sin(m**(-1)*F.sqrt((-1)* b**2+2*k*m)*tstpfp)))))
    eqdd0 = ((1/2)*((-1/2)*gff1+(1/2)*F.exp((-1)*b*m**(-1)*tstpfp)*l0*((-1)* b**2+
        2*k*m)**(-1/2)*((-1)*F.sqrt((-1)*b**2+2*k*m)*F.cos(m**( -1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+(((-8)+
        gsumafp**2)*l0+8* yapex)**(-3/2)*(16*F.sqrt(gy)*m*(l0+(-1)*yapex)+(-1)*b*(((-8) +gsumafp**2)*l0+
        8*yapex)**(3/2))*F.sin(m**(-1)*F.sqrt((-1)* b**2+2*k*m)*tstpfp))+
        (1/16)*F.exp((-2)*b*m**(-1)*tstpfp)*gy**(-1) *(b**2+(-2)*k*m)**(-1)*(((-8)+gsumafp**2)*l0+
        8*yapex)**( -3/2)*(F.exp(b*m**(-1)*tstpfp)*((-4)*gfg2*gsumafp+gfg1*(( -8)+
        gsumafp**2))*F.sqrt((-1)*b**2+2*k*m)+4*F.sqrt(gy)*F.sqrt(( -1)*b**2+2*k*m)*F.sqrt(((-8)+
        gsumafp**2)*l0+8*yapex)*F.cos( m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+(-2)*(((-8)+ gsumafp**2)*k*l0+
        (-4)*gy*m+2*b*F.sqrt(gy)*F.sqrt(((-8)+ gsumafp**2)*l0+8*yapex))*F.sin(m**(-1)*F.sqrt((-1)*b**2+
        2*k* m)*tstpfp))*((-1)*F.exp(b*m**(-1)*tstpfp)*gfg1*F.sqrt((-1)* b**2+2*k*m)*(((-8)+gsumafp**2)*l0+
        8*yapex)**(3/2)+16*F.sqrt( gy)*l0*F.sqrt((-1)*b**2+2*k*m)*(l0+
        (-1)*yapex)*F.cos(m**(-1)* F.sqrt((-1)*b**2+2*k*m)*tstpfp)+(-2)*l0*(8*b*F.sqrt(gy)*(l0+( -1)*yapex)+
        (-1)*k*(((-8)+gsumafp**2)*l0+8*yapex)**(3/2))* F.sin(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp))+
        (1/8)*gy**(-1)*( (-2)*gfg2+gfg1*gsumafp+2*F.exp((-1)*b*m**(-1)*tstpfp)* gsumafp*l0*((-1)*b**2+
        2*k*m)**(-1/2)*(((-8)+gsumafp**2) *l0+8*yapex)**(-1/2)*(F.sqrt(gy)*F.sqrt((-1)*b**2+
        2*k*m)*F.cos( m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+(-1)*(b*F.sqrt(gy)+k* F.sqrt(((-8)+
        gsumafp**2)*l0+8*yapex))*F.sin(m**(-1)*F.sqrt((-1)* b**2+2*k*m)*tstpfp)))**2))
    # =========================================================================%
    gsum = eqdd0**(-1)*((-1)*eqd0+F.sqrt(eqd0**2+(-2)*eq0*eqdd0)+eqdd0* gsumafp)
    # -------------------------------------------------------------------------%
    # here, there is a choice of using h or yapex.
    dxddes = xddesmod-xdtd
    gambdm = -xdrl/l0
    tfl1 = (F.sqrt(2*gy*yapex-2*gy*l0*F.cos(gsum/2)))/gy
    ytd = yapex-0.5*gy*tfl1**2
    ydtd = -gy*tfl1
    # -------------------------- torb, torf calculation -----------------------%
    c1 = (-1/2)*gy*k**(-1)*m
    c2 = ((-2)*k*l0**2*(l0+spl)*((-2)*k*l0**2+m*xdrl**2)*(4*k**2* l0**4+4*l0**2*(b**2+(-1)*k*m)*xdrl**2+
        m**2*xdrl**4)**(-1)*F.cos(( 1/2)*gsum)+(-4)*b*k*l0**3*(l0+spl)*xdrl*(4*k**2*l0**4+4* l0**2*(b**2+
        (-1)*k*m)*xdrl**2+m**2*xdrl**4)**(-1)*F.sin((1/2)* gsum))
    c3 = ((1/8)*k**(-1)*(4*k**2*l0**4+4*l0**2*(b**2+(-1)*k*m)* xdrl**2+m**2*xdrl**4)**(-1)*(((-1)*((-8)+
        gsum**2)*k*l0+4*gy* m)*(4*k**2*l0**4+4*l0**2*(b**2+(-1)*k*m)*xdrl**2+m**2* xdrl**4)+16*k**2*l0**2*(l0+
        spl)*(((-2)*k*l0**2+m*xdrl**2)*F.cos( (1/2)*gsum)+2*b*l0*xdrl*F.sin((1/2)*gsum))))
    c4 = ((1/2)*k**(-1)*((-1)*b**2+2*k*m)**(-1/2)*((-1/4)*b*((-8)+ gsum**2)*k*l0+b*gy*m+
        (-1)*F.sqrt(gy)*k*m*F.sqrt(((-8)+ gsum**2)*l0+8*yapex)+4*k**2*l0*(l0+spl)*(4*k**2*l0**4+
         4*l0**2*(b**2+(-1)*k*m)*xdrl**2+m**2*xdrl**4)**(-1)*((-1)*b* l0*(2*k*l0**2+
        m*xdrl**2)*F.cos((1/2)*gsum)+xdrl*(2*l0**2*( b**2+(-1)*k*m)+m**2*xdrl**2)*F.sin((1/2)*gsum))))
    c5 = (4*b*k*l0**3*(l0+spl)*xdrl*(4*k**2*l0**4+4*l0**2*(b**2+(-1) *k*m)*xdrl**2+
        m**2*xdrl**4)**(-1)*F.cos((1/2)*gsum)+4*k**2* l0**4*(l0+spl)*(4*k**2*l0**4+4*l0**2*(b**2+
        (-1)*k*m)* xdrl**2+m**2*xdrl**4)**(-1)*F.sin((1/2)*gsum)+(-2)*k*l0**2*m*( l0+
        spl)*xdrl**2*(4*k**2*l0**4+4*l0**2*(b**2+(-1)*k*m)* xdrl**2+m**2*xdrl**4)**(-1)*F.sin((1/2)*gsum))
    torb = ((1/2)*l0*m*tstpfp**(-1)*((-1)*dxddes+(1/2)*c4*gsum*m**( -1)*F.sqrt((-1)*b**2+2*k*m)+
        2*c2*k*l0*m**(-1)*xdrl**(-1)+c5* gsum*k*l0*m**(-1)*xdrl**(-1)+c3*l0**(-1)*xdrl+
        (-1)*b**2*c3* k**(-1)*l0**(-1)*m**(-1)*xdrl+(1/2)*b*k**(-1)*l0**(-1)*m**( -1)*(c3*gsum*k*l0+
        (-2)*c4*F.sqrt((-1)*b**2+2*k*m)*xdrl)+( -1)*k*l0**(-1)*m**(-1)*((-1)*c1+l0+spl)*tstpfp*(gsum*l0+
         (-1)*tstpfp*xdrl)+(-1/2)*F.exp((-1)*b*m**(-1)*tstpfp)*k**(-1)* l0**(-1)*m**(-1)*(b*c3*gsum*k*l0+
        c4*gsum*k*l0*F.sqrt(( -1)*b**2+2*k*m)+(-2)*b**2*c3*xdrl+2*c3*k*m*xdrl+(-2)*b* c4*F.sqrt((-1)*b**2+
        2*k*m)*xdrl+(-2)*b*c3*k*tstpfp*xdrl+(-2)* c4*k*F.sqrt((-1)*b**2+
        2*k*m)*tstpfp*xdrl)*F.cos(m**(-1)*F.sqrt(( -1)*b**2+2*k*m)*tstpfp)+k*m**(-1)*xdrl**(-1)*((-2)*c2*l0+
        ( -1)*c5*gsum*l0+2*c5*tstpfp*xdrl)*F.cos(l0**(-1)*tstpfp*xdrl)+
         (1/2)*c3*F.exp((-1)*b*m**(-1)*tstpfp)*gsum*m**(-1)*F.sqrt(( -1)*b**2+
        2*k*m)*F.sin(m**(-1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp) +
        (-1)*b*c3*F.exp((-1)*b*m**(-1)*tstpfp)*k**(-1)*l0**(-1)* m**(-1)*F.sqrt((-1)*b**2+
        2*k*m)*xdrl*F.sin(m**(-1)*F.sqrt((-1)* b**2+2*k*m)*tstpfp)+
        (-1)*c3*F.exp((-1)*b*m**(-1)*tstpfp)* l0**(-1)*m**(-1)*F.sqrt((-1)*b**2+
        2*k*m)*tstpfp*xdrl*F.sin(m**( -1)*F.sqrt((-1)*b**2+2*k*m)*tstpfp)+
        (-1/2)*c4*F.exp((-1)*b* m**(-1)*tstpfp)*k**(-1)*l0**(-1)*m**(-1)*(b*gsum*k*l0+( -2)*b**2*xdrl+
        2*k*m*xdrl+(-2)*b*k*tstpfp*xdrl)*F.sin(m**(-1)* F.sqrt((-1)*b**2+2*k*m)*tstpfp)+
        (-1)*k*m**(-1)*xdrl**(-1)*(2* c5*l0+(-1)*c2*gsum*l0+2*c2*tstpfp*xdrl)*F.sin(l0**(-1)* tstpfp*xdrl)))
    torf = torb
    # ------------------------------------------------------------------------%
    tfl = 2*F.sqrt(2*gy*(h-l0*F.cos(xddes*tstpfp/(2*l0))))/gy
    # (after leg positioning)
    amlo = (gambdlo*mt**(-1)*((-1)*l1**2*ml**2+il*mt+l1**2*ml*((-1)* ml+mt))+
        gamfdlo*mt**(-1)*((-1)*l1**2*ml**2+il*mt+l1**2*ml*( (-1)*ml+mt))+(ib+2*d**2*ml)*thdlo)
    thdalp = amlo*(ib+2*d**2*ml)**(-1)
    thalp = ((ib+2*d**2*ml)**(-1)*mt**(-1)*(2*(gsumpfp/2)*l1**2*ml**2+(-2)* gamflo*l1**2*ml**2+
        2*(gsumpfp/2)*l1**2*ml**2+gamflo*il*mt+(-1)* (gsumpfp/2)*il*mt+gamflo*l1**2*ml*mt+
        (-1)*(gsumpfp/2)*l1**2*ml*mt+( -1)*(gsumpfp/2)*(il+l1**2*ml)*mt+gamblo*(il*mt+l1**2*ml*((-2)* ml+mt))+
        amlo*mt*(tfl/2)+(ib+2*d**2*ml)*mt*thlo))
    # Uncomment the following line to effectively turn off the reaction wheel.
    kpw=1e-9
    kdw=0
    thtd = (F.exp((-1)*kpw*(ib*kpw)**(-1/2)*(tfl/2))*ib**(-1)*(F.sqrt(ib*kpw)* (tfl/2)*thalp+
        ib*((tfl/2)*thdalp+thalp)))
    thdtd = (F.exp((-1)*kpw*(ib*kpw)**(-1/2)*(tfl/2))*ib**(-1)*(ib*thdalp+F.sqrt( ib*kpw)*thalp)+
        (-1)*F.exp((-1)*kpw*(ib*kpw)**(-1/2)*(tfl/2))* ib**(-1)*kpw*(ib*kpw)**(-1/2)*(F.sqrt(ib*kpw)*(tfl/2)*thalp+
        ib*( (tfl/2)*thdalp+thalp)))
    a0 = (F.exp((-1)*b*d**2*ib**(-1)*tstpfp)*((-1)*b**2*d**4+2*d**2* ib*k)**(-1/2)*(F.sqrt((-1)*b**2*d**4+
        2*d**2*ib*k)*thdtd* F.cos(ib**(-1)*F.sqrt((-1)*b**2*d**4+2*d**2*ib*k)*tstpfp)+(-1) *(b*d**2*thdtd+
        2*d**2*k*thtd+torb+torf)*F.sin(ib**(-1)*F.sqrt( (-1)*b**2*d**4+2*d**2*ib*k)*tstpfp)))
    a1 = (2*d*F.exp((-1)*b*d**2*ib**(-1)*tstpfp)*k*((-1)*b**2*d**4+ 2*d**2*ib*k)**(-1/2)*l0*(l0+
        spl)*(4*d**4*k**2*l0**4+4* d**2*(b**2*d**2+(-1)*ib*k)*l0**2*xdrl**2+
        ib**2*xdrl**4)**(-1)* (F.sin((1/2)*gsum)*((-2)*b*d**2*F.sqrt((-1)*b**2*d**4+
        2*d**2* ib*k)*l0*xdrl**2*F.cos(ib**(-1)*F.sqrt((-1)*b**2*d**4+2*d**2* ib*k)*tstpfp)+
        2*b*d**2*F.exp(b*d**2*ib**(-1)*tstpfp)*F.sqrt(( -1)*b**2*d**4+
        2*d**2*ib*k)*l0*xdrl**2*F.cos(l0**(-1)*tstpfp* xdrl)+2*d**2*l0*(2*d**2*k**2*l0**2+(b**2*d**2+
        (-1)*ib*k)* xdrl**2)*F.sin(ib**(-1)*F.sqrt((-1)*b**2*d**4+2*d**2*ib*k)* tstpfp)+
        (-2)*d**2*F.exp(b*d**2*ib**(-1)*tstpfp)*k*F.sqrt((-1)* b**2*d**4+
        2*d**2*ib*k)*l0**2*xdrl*F.sin(l0**(-1)*tstpfp*xdrl)+
         F.exp(b*d**2*ib**(-1)*tstpfp)*ib*F.sqrt((-1)*b**2*d**4+
        2* d**2*ib*k)*xdrl**3*F.sin(l0**(-1)*tstpfp*xdrl))+
        (-1)*xdrl*F.cos(( 1/2)*gsum)*(F.sqrt((-1)*b**2*d**4+2*d**2*ib*k)*((-2)*d**2* k*l0**2+
        ib*xdrl**2)*F.cos(ib**(-1)*F.sqrt((-1)*b**2*d**4+2*d**2* ib*k)*tstpfp)+
        F.exp(b*d**2*ib**(-1)*tstpfp)*F.sqrt((-1)*b**2* d**4+2*d**2*ib*k)*(2*d**2*k*l0**2+
        (-1)*ib*xdrl**2)*F.cos( l0**(-1)*tstpfp*xdrl)+b*d**2*(((-2)*d**2*k*l0**2+
        (-1)*ib* xdrl**2)*F.sin(ib**(-1)*F.sqrt((-1)*b**2*d**4+2*d**2*ib*k)* tstpfp)+
        2*F.exp(b*d**2*ib**(-1)*tstpfp)*F.sqrt((-1)*b**2*d**4+
         2*d**2*ib*k)*l0*xdrl*F.sin(l0**(-1)*tstpfp*xdrl)))))
    thdlodes = (-(gambdlo*mt**(-1)*((-1)*l1**2*ml**2+il*mt+l1**2*ml*((-1)* ml+mt))+
        gamfdlo*mt**(-1)*((-1)*l1**2*ml**2+il*mt+l1**2*ml*( (-1)*ml+mt)))/(ib+2*d**2*ml))
    gtd = -2*F.asin((a0-thdlodes)/(a1))
    torbctr=torb
    # if abs(gtd)<0.1;   gdif = gtd;   else;   gdif=sign(gtd)*0.1;   end;
    gdif=gtd
    # ----------------------- 2. THANASIS ----------------------------------------&
    # Achieve during flight phase
    gambtd = (gsum+gdif)/2
    gamftd = (gsum-gdif)/2
    # Use during stance (front, back or double) phase
    torfctr = torbctr
    out = [gambtd,gamftd,torbctr,torfctr]
    # ----------------------- 2. THANASIS ----------------------------------------&
    return out


def gtd_controller8(m, ib, mt, mul, ml, il, l1, d, l0, g, k, b, spl, psia,
                    xddes, h, xdlo, ylo, ydlo, thlo, thdlo, gamblo, gamflo, gambdlo, gamfdlo):
    """Return [gambtd, gamftd, torbctr, torfctr]: touchdown leg angles and stance torques."""
    args = (m, ib, mt, mul, ml, il, l1, d, l0, g, k, b, spl, psia,
            xddes, h, xdlo, ylo, ydlo, thlo, thdlo, gamblo, gamflo, gambdlo, gamfdlo)
    try:
        out = _gtd_controller8_body(math, *args)
    except (ValueError, TypeError):
        out = _gtd_controller8_body(cmath, *args)
    return _real(out, 'gtd_controller8')
