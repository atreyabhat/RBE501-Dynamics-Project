function J = jacob0(S,q) 
    [~,col] = size(S);
    J = zeros(6,col);
        
    J(:,1) = S(:,1);
    for i=2:col
        T = eye(4);
        for j = 2:i
            T = T * twist2ht(S(:,j-1),q(j-1));
        end
        J(:,i) = adjoint(S(:,i),T);
    end
end