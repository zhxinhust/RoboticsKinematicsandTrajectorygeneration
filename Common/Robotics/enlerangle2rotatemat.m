function g = enlerangle2rotatemat(p, AER)

AER = AER * pi / 180;
A1 = [cos(AER(1)), -sin(AER(1)), 0;
      sin(AER(1)), cos(AER(1)), 0;
      0, 0, 1];

A2 = [cos(AER(2)), 0, sin(AER(2));
      0, 1, 0;
      -sin(AER(2)), 0, cos(AER(2))];
  
A3 = [cos(AER(3)), -sin(AER(3)), 0;
      sin(AER(3)), cos(AER(3)), 0;
      0, 0, 1];
R = A1 * A2 * A3;

g = eye(4);
g(1:3, 1:3) = R;
g(1, 4) = p(1);
g(2, 4) = p(2);
g(3, 4) = p(3);