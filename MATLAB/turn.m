function [moti_billi] = turn(moti_billi,theta,h)
%h = figure('Position', [10 10 1000 1000]);
% heading angle of quadruped
plotBody(moti_billi, h, false);
%%rotation specs
if theta>=0 
    step_angle=pi/24;
else
    step_angle=-pi/24;
end
%angle=step_angle;
num_steps = abs(theta/step_angle);%%replace it with the "desired_turning_angle/step_angle"
%%applying cycloid trajectory to legs%%
for i = 1:num_steps
meow=moti_billi.rotate(step_angle);
init_feet = [moti_billi.frontRight.endPoint, moti_billi.frontLeft.endPoint, moti_billi.backRight.endPoint, moti_billi.backLeft.endPoint]; 
final_feet = [meow.frontRight.endPoint, meow.frontLeft.endPoint, meow.backRight.endPoint, meow.backLeft.endPoint]; %%updated position of cat
dist_frbl = sqrt((final_feet(1,1)-init_feet(1,1))^2 + (final_feet(2,1)-init_feet(2,1))^2);%%distance to be moved by front right and back left legs
dir_frbl = [(final_feet(1,1)-init_feet(1,1)) (final_feet(2,1)-init_feet(2,1)) ]./dist_frbl;%%direction unit vector from prev. position of front right foot to current positon 
dist_flbr = sqrt((final_feet(1,2)-init_feet(1,2))^2 + (final_feet(2,2)-init_feet(2,2))^2);%%distance to be moved by front left and back right legs
dir_flbr = [(final_feet(1,2)-init_feet(1,2)) (final_feet(2,2)-init_feet(2,2)) ]./dist_flbr;%%direction unit vector from prev. position of front left foot to current positon 
%%
%% define step specs
stepLength = dist_frbl; % max range of foot for front right and back left

%% define walk parameters
t_step = 0.1;     % 1 second for 1 footstep

num_its = 20;   % no of substeps in one footstep
c_step = t_step/num_its;


%% describing walk motion
    init_feet = [moti_billi.frontRight.endPoint, moti_billi.frontLeft.endPoint, moti_billi.backRight.endPoint, moti_billi.backLeft.endPoint]; 
    init_com = moti_billi.com;
%%updating orientation
    moti_billi = moti_billi.update( init_feet(:,1),... % location of fr.endPoint
                                        init_feet(:,2),...                            % location of fl.endPoint
                                        init_feet(:,3),...                            % location of br.endPoint
                                        init_feet(:,4),... % location of bl.endPoint
                                        init_com,...                                     % location of CoM
                                        moti_billi.zero_angle+step_angle);
%%end updating orientation

    % first half
    for t = linspace(0, t_step, num_its)
        x_foot = (stepLength/pi)*0.5*(t/t_step*2*pi - sin(t/t_step*2*pi))*dir_frbl(1);
        y_foot = (stepLength/pi)*0.5*(t/t_step*2*pi - sin(t/t_step*2*pi))*dir_frbl(2);
        z_foot = 0.5*(1 - cos(t/t_step*2*pi));
        
        moti_billi = moti_billi.update( init_feet(:,1) + [x_foot;y_foot; z_foot],... % location of fr.endPoint
                                        init_feet(:,2),...                            % location of fl.endPoint
                                        init_feet(:,3),...                            % location of br.endPoint
                                        init_feet(:,4) + [-x_foot; -y_foot; z_foot],... % location of bl.endPoint
                                        init_com,...                                     % location of CoM
                                        moti_billi.zero_angle);
        
        plotBody(moti_billi, h, false);
        pause(c_step);
    end
    %%second half
    stepLength=dist_flbr;%%for front left and baack right
    init_feet = [moti_billi.frontRight.endPoint, moti_billi.frontLeft.endPoint, moti_billi.backRight.endPoint, moti_billi.backLeft.endPoint]; 
    init_com = moti_billi.com;
    for t = linspace(0, t_step, num_its)

        x_foot = (stepLength/pi)*0.5*(t/t_step*2*pi - sin(t/t_step*2*pi))*dir_flbr(1);
        y_foot = (stepLength/pi)*0.5*(t/t_step*2*pi - sin(t/t_step*2*pi))*dir_flbr(2);
        z_foot = 0.5*(1 - cos(t/t_step*2*pi));
        
        moti_billi = moti_billi.update( init_feet(:,1),...                            % location of fr.endPoint
                                        init_feet(:,2) + [x_foot; y_foot; z_foot],... % location of fl.endPoint
                                        init_feet(:,3) + [-x_foot; -y_foot; z_foot],... % location of br.endPoint
                                        init_feet(:,4),...                            % location of bl.endPoint
                                        init_com, ...                                    % location of CoM
                                        moti_billi.zero_angle);
        plotBody(moti_billi, h, false);
 
        pause(c_step);
    end
    %angle= angle+step_angle;%%orientation angle for billi in next loop
end
end