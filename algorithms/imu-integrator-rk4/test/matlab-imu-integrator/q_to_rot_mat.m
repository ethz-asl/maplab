function R = q_to_rot_mat(q)
w = q(1);
x = q(2);
y = q(3);
z = q(4);

Rxx = 1 - 2*(y^2 + z^2);
Rxy = 2*(x*y - z*w);
Rxz = 2*(x*z + y*w);

Ryx = 2*(x*y + z*w);
Ryy = 1 - 2*(x^2 + z^2);
Ryz = 2*(y*z - x*w );

Rzx = 2*(x*z - y*w );
Rzy = 2*(y*z + x*w );
Rzz = 1 - 2 *(x^2 + y^2);

% Typical qToRotMat is transposed as we use passive rotations.
R = [Rxx,    Rxy,    Rxz;
     Ryx,    Ryy,    Ryz;
     Rzx,    Rzy,    Rzz]';