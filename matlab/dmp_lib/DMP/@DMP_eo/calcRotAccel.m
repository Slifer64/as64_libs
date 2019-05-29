function rotAccel = calcRotAccel(this, x, Q, Qg, Q0, rotVel, tau_dot, yc, zc, yc_dot)
            
    if (nargin < 7), tau_dot = 0; end
    if (nargin < 8), yc = zeros(3,1); end
    if (nargin < 9), zc = zeros(3,1); end
    if (nargin < 10), yc_dot = zeros(3,1); end
    
%     Qe = DMP_eo.quatError(Q, Qg);
%     Y = DMP_eo.quat2eo(Q, Qg);
%     Y0 = DMP_eo.quat2eo(Q0, Qg);
%     deo = DMP_eo.rotVel2deo(rotVel, Qe);
%     Z = this.getTau()*deo - yc;
%     
%     disp('========== [DMP_eo::calcRotAccel] ============');
%     x
%     Yi = Y'
%     Zi = Z'
%     Y0i = Y0'
% 
%     for i=1:3, this.dmp{i}.update(x, Y(i), Z(i), Y0(i), 0, yc(i), zc(i)); end
% 
%     dY = zeros(3,1);
%     dZ = zeros(3,1);
%     for i=1:3
%         dY(i) = this.dmp{i}.getYdot();
%         dZ(i) = this.dmp{i}.getZdot();    
%     end
%     
%     ddeo = (dZ - tau_dot*dY + yc_dot)/this.getTau(); 
%     deo = dY; 
%     rotVel = DMP_eo.deo2rotVel(deo, Qe);
%     rotAccel = DMP_eo.ddeo2rotAccel(ddeo, rotVel, Qe);
%     
%     return;

    a_z = [this.dmp{1}.getAz(); this.dmp{2}.getAz(); this.dmp{3}.getAz()];
    b_z = [this.dmp{1}.getBz(); this.dmp{2}.getBz(); this.dmp{3}.getBz()];
    tau = this.getTau();

    Qe = DMP_eo.quatError(Q, Qg);
    eo = DMP_eo.quat2eo(Q, Qg);
    invQe = quatInv(Qe);

    rotVelQ = [0; rotVel];
    QeRotVel = quatProd(Qe,rotVelQ);

    J_deo_dQ = DMP_eo.jacobDeoDquat(Qe);
    J_dQ_deo = DMP_eo.jacobDquatDeo(Qe);
    dJ_dQ_deo = DMP_eo.jacobDotDquatDeo(Qe, rotVel);

    fo = zeros(3,1);
    eo0 = DMP_eo.quat2eo(Q0,Qg);
    for i=1:3, fo(i) = this.dmp{i}.shapeAttractor(x, eo0(i), 0); end

    deo = DMP_eo.rotVel2deo(rotVel, Qe);
    ddeo = (-a_z.*b_z.*eo - tau*a_z.*deo + a_z.*yc + fo + tau*yc_dot - tau*tau_dot*deo + zc)/tau^2;

    rotAccel1 = quatProd(invQe, dJ_dQ_deo*J_deo_dQ*QeRotVel);
    % rotAccel2 = 2*quatProd(invQe, J_dQ_deo* (a_z.*b_z.*eo - 0.5*tau*a_z.*(J_deo_dQ*QeRotVel) - fo) ) / tau^2;
    rotAccel2 = 2*quatProd(invQe, J_dQ_deo*-ddeo);   
    rotAccel = rotAccel1 + rotAccel2;  
    rotAccel = rotAccel(2:4);

%             norm(rotAccel-rotAccel_2)

end