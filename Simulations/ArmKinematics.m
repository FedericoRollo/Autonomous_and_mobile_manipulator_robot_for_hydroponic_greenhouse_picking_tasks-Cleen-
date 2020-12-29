
function [k_q, j_q, j_p] = ArmKinematics()
    syms d1 a2 a3 d4 d5 d6 al1 al4 al5
    q1 = sym('q1');
    q2 = sym('q2');
    q3 = sym('q3');
    q4 = sym('q4');
    q5 = sym('q5');
    q6 = sym('q6');

    q = [q1 q2 q3 q4 q5 q6];

    % sostituire gli 1 con valori reali dalla simulazione.
    DH_param = [q1 0        0      pi/2;
                q2 0.216    0      0;
                q3 0        0      pi/2;
                q4 0        0.219  -pi/2;
                q5 0        0      pi/2;
                q6 0        0.198  0];




    A={zeros(4);zeros(4);zeros(4);zeros(4);zeros(4);zeros(4)};
    A0 = [1 0 0 0; 0 1 0 0; 0 0 1 0.283; 0 0 0 1];

    for i = 1:6
        qi = DH_param(i,1);
        a = DH_param(i,2);
        d = DH_param(i,3);
        al = DH_param(i,4);

        A{i}=[cos(qi) -sin(qi)*cos(al) sin(qi)*sin(al) a*cos(qi);
              sin(qi) cos(qi)*cos(al) -cos(qi)*sin(al) a*sin(qi);
              0 sin(al) cos(al) d;
              0 0 0 1];

    end

    %Direct kinematic in homogeneous transformation

    f_q = A0*A{1}*A{2}*A{3}*A{4}*A{5}*A{6};

    %Direct kinematic in state vector [x y z phi th psi] using ZYZ angles
    x_q = f_q(1,4);
    y_q = f_q(2,4);
    z_q = f_q(3,4);

    phi = atan2(f_q(2,3),f_q(1,3));
    th = atan2(sqrt(f_q(1,3)^2+f_q(2,3)^2),f_q(3,3));
    psi = atan2(f_q(3,2),f_q(3,1));

    k_q = [x_q; y_q; z_q; phi; th; psi];

    j_q = jacobian(k_q, [q1 q2 q3 q4 q5 q6]);

    j_p = j_q(1:3,:);

    j_a = j_q(4:6,:);
    
end
