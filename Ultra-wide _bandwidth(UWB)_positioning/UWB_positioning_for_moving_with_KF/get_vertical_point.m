function [ vertical_point ] = get_vertical_point(X, X_act, k_decide)

k = -2 * logical(k_decide) + 1;
A = [-k 1;1/k 1];
B = [-k*X_act(1)+X_act(2) 1/k*X(1)+X(2)]';
vertical_point = A\B;

end