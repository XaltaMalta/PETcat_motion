function [moti_billi] = locomote(moti_billi,list)
h = figure('Position', [10 10 1000 1000]);
ncol = size(list,2)

for i = 1:ncol
    point = list(:,i);
    theta = atan((point(2)-moti_billi.com(2))/((point(1)-moti_billi.com(1))));
    s1 = (point(2)-moti_billi.com(2));%sign if sine
    %s2 = (point(1)-moti_billi.com(1));%sign of cosine
    if theta<0 && s1>0
        theta = pi+theta;
    elseif theta>0 && s1<0  
        theta = pi+theta;
    end
    turning_angle = theta - moti_billi.zero_angle;
    moti_billi = turn(moti_billi,turning_angle,h);
    walk_dist = sqrt((point(1)-moti_billi.com(1))^2+(point(2)-moti_billi.com(2))^2);
    moti_billi = walk(moti_billi,walk_dist,h);
end
