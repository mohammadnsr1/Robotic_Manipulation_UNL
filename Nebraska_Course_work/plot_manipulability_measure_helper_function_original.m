clc
clear all
close all

s =@sin;
c =@cos;

% vars
d1=110;
l2=105;
l3=150;

% joint range
t1=0;
t2_range=linspace(0,pi,100);
t3_range=linspace(-90/180*pi,0,50);

idx1=1; idx2=1;
for t2 = t2_range
    for t3 = t3_range
        % computer the forward kinematics
        x(idx1,idx2) = c(t1)*(l2*c(t2)+l3*c(t2-t3));
        y(idx1,idx2) = s(t1)*(l2*c(t2)+l3*c(t2-t3));
        z(idx1,idx2) = d1+l2*s(t2)+l3*s(t2-t3);
        
        % computer the analytic Jacobian
        J =[] ;
        
        % compute the manipulability measure
        A = ;
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
view(0,90)

xlabel('P_x','FontWeight','bold')
ylabel('P_z','FontWeight','bold')
set(gca,'FontSize',18)
box on