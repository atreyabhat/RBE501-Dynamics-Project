function withinLimits = checklimits(currentQ)
    % Check if each joint value is within the corresponding limits in qlim
    qlim = [-170 170;
           -120 120;
           -170 170;
           -120 120;
           -170 170;
           -120 120;
           -175 175];
    withinLimits = all(currentQ >= qlim(:,1)') && all(currentQ <= qlim(:,2)');
end

