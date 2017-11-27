function R = skew3(w)

R = zeros(3,3);
R(1,2) = -w(3);
R(1,3) =  w(2);
R(2,3) = -w(1);

R(2,1) =  w(3);
R(3,1) = -w(2);
R(3,2) =  w(1); 

end