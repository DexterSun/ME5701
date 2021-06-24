u_a = u-Wind_Vx_body;
v_a = v-Wind_Vy_body;
w_a = w-Wind_Vz_body;

Va = sqrt((u_a)^2+(v_a)^2+(w_a)^2);

Alpha = atan(w_a/u_a);
Beta = atan(v_a/Va);

Qbar = 0.5 * Rho * Va^2;

Qbar_coff = sqrt(Qbar_base / Qbar);

ParaLimit(Qbar_coff,0.33,3);

CD = CD_0 ...
    + CD_Alpha * Alpha ...
    + CD_Beta * Beta ...
    + CD_De * De ...
    + CD_Da * Da ...
    + CD_Dr * Dr;

CY = CY_0 ...
    + CY_Beta * Beta ...
    + CY_Da * Da ...
    + CY_Dr * Dr;

CL = CL_0 ...
    + CL_Alpha * Alpha ...
    + CL_Da * Da ...
    + CL_De * De ...
    + CL_Dr * Dr;
    
Cl = Cl_Beta *Beta ...
    + Cl_p * Span * p / V0 / 2 ...
    + Cl_Da * Da ...
    + Cl_Dr * Dr;

Cm = Cm_0 ...
    + Cm_Alpha * Alpha ...
    + Cm_q * q * Chord / V0 / 2 ...
    + Cm_De * De;

Cn = Cn_Beta * Beta ...
    + Cn_p * p * Span / V0 / 2 ...
    + Cn_r * r * Span / V0 / 2 ...
    + Cn_Dr * Dr ...
    + Cn_Da * Da;
T = CT*DT;
D = Qbar * Sref * CD;
L = Qbar * Sref * CL;
Y = Qbar * Sref * CY;
Ml = Qbar * Sref * Cl * Span;
Mm = Qbar * Sref * Cm * Chord - T * Mm_L;
Mn = Qbar * Sref * Cn * Span;



