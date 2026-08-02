clc
clear all
close all

s =@sin;
c =@cos;

% vars
L1=110;
L2=105;
L3=150;

% joint range
t1=0;
t2_range=linspace(0,pi,100);
t3_range=linspace(0,90/180*pi,50);

idx1=1; idx2=1;
for t2 = t2_range
    for t3 = t3_range
        % computer the forward kinematics
        x(idx1,idx2) = c(t1)*(L2*c(t2)+L3*c(t2-t3));
        y(idx1,idx2) = s(t1)*(L2*c(t2)+L3*c(t2-t3));
        z(idx1,idx2) = L1+L2*s(t2)+L3*s(t2-t3);
        
        % computer the analytic Jacobian
        J = [-sin(t1)*(L2*cos(t2) + L3*cos(t2 - t3)), -cos(t1)*(L2*sin(t2) + L3*sin(t2 - t3)), L3*sin(t2 - t3)*cos(t1);
             cos(t1)*(L2*cos(t2) + L3*cos(t2 - t3)), -sin(t1)*(L2*sin(t2) + L3*sin(t2 - t3)), L3*sin(t2 - t3)*sin(t1);
                                    0,            L2*cos(t2) + L3*cos(t2 - t3),        -L3*cos(t2 - t3)];
        
        % compute the manipulability measure
        A = J*J';
        mu3(idx1,idx2) = real(sqrt(det(A)));
                
        idx2 = idx2+1;
    end
    idx1 = idx1+1;
    idx2=1;
end

%% Plot manipulability measure as a function of the workspace
colormap(jet)
norm_mu3 = mu3/max(mu3(:));
surf(x,z,norm_mu3)
shading interp
colorbar
% view(0,90)

xlabel('P_x','FontWeight','bold')
ylabel('P_z','FontWeight','bold')
set(gca,'FontSize',18)
box on