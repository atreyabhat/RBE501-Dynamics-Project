function T = fkine(S,M,q,frame)    
    [~,joints] = size(S);
    
    if strcmp(frame,'space')
        for i=joints:-1:1
            if i ~= joints
                T = twist2ht(S(:,i),q(i))*T;
            else
                T = twist2ht(S(:,i),q(i))*M;
            end
        end

    elseif strcmp(frame,'body')
        T = M * twist2ht(S(:,1),q(1));
        for i=2:joints
            T = T * twist2ht(S(:,i),q(i));
        end
    end
end