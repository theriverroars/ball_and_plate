function delta = inv_kinematics(alpha, beta, B, P_p, Tb, Rm, D, rt_)
    Rx= [1 0 0; 0 cos(beta) sin(beta); 0 -sin(beta) cos(beta)];
    Ry = [cos(alpha) 0 sin(alpha); 0 1 0; -sin(alpha) 0 cos(alpha)];
    Rb = Ry*Rx;
    
    P = P_p*Rb'+ Tb
    L = P - B;
    norm
    
    a = 2*Rm*(P(:,3) - B(:,3));
    b = 2*Rm*((P(:,1)- B(:,1)).*cos(rt_)' + (P(:,2)- B(:,2)).*sin(rt_)');
    c = sum(L.^2,2) - D^2 + Rm^2;
    delta = asin(c./sqrt(a.^2 +b.^2)) - atan(b./a);
    delta = rad2deg(delta);
end