% V = kS*sgn(v) + kV*v + kA*a
% v = (vL + vR)/2
% According to control engineering for FRC book...
% (creatively named variables I know)
% aL = (C1*vL + C2*VL)
function xDot = robotSystemUKF_deriv(x, u)
    physicsConstants;
    
    xDot = zeros(STATE_SIZE, 1);
    
    xDot(X) = (x(vL) + x(vR))/2*cos(x(THETA));
    xDot(Y) = (x(vL) + x(vR))/2*sin(x(THETA));
    xDot(THETA) = 1/(2*rb)*(x(vR) - x(vL));
    xDot(vL) = (Am*C1*x(vL) + Am*C2*u(VL)) + (Bm*C1*x(vR) + Bm*C2*u(VR));
    xDot(vR) = (Am*C1*x(vR) + Am*C2*u(VR)) + (Bm*C1*x(vL) + Bm*C2*u(VL));
end

