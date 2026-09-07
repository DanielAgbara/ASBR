function q = RToQuaternion(R)
    q0 = sqrt(R(1,1) + R(2,2) + R(3,3) + 1) / 2;
    sq1 = max(0, R(1,1) - R(2,2) - R(3,3) + 1);
    sq2 = max(0, R(2,2) - R(3,3) - R(1,1) + 1);
    sq3 = max(0, R(3,3) - R(1,1) - R(2,2) + 1);

    q1 = sign(R(3,2) - R(2,3)) * sqrt(sq1) / 2;
    q2 = sign(R(1,3) - R(3,1)) * sqrt(sq2) / 2;
    q3 = sign(R(2,1) - R(1,2)) * sqrt(sq3) / 2;

    q = [q0; q1; q2; q3];
    q = q / norm(q);
end