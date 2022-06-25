clear all;

m_A = 1;
m_B = 1;
m_C = 1;
m_D = 1;
g = [0;-9.8];
C = 0.03;
dt = 0.01;
k = 0.1;

%% memory allocate

current_x_A = [0;100];
current_x_B = [0;99];
current_x_C = [2;100];
current_x_D = [2;97];
current_v_A = [0;0];
current_v_B = [0;0];
current_v_C = [0;0];
current_v_D = [0;0];
L_0 = 1;
L_d = sqrt(2*L_0^2);
next_x_A = [0;0];
next_x_B = [0;0];
next_x_C = [0;0];
next_x_D = [0;0];
next_v_A = [0;0];
next_v_B = [0;0];
next_v_C = [0;0];
next_v_D = [0;0];
%%

F_gravity_A = m_A * g;
F_gravity_B = m_B * g;
F_gravity_C = m_C * g;
F_gravity_D = m_D * g;
F_damp_A = -C*current_v_A;
F_damp_B = -C*current_v_B;
F_damp_C = -C*current_v_C;
F_damp_D = -C*current_v_D;

dv_A = F_gravity_A+F_damp_A;
dv_B = F_gravity_B+F_damp_B;
dv_C = F_gravity_C+F_damp_C;
dv_D = F_gravity_D+F_damp_D;

next_v_A = current_v_A + dv_A/m_A;
next_v_B = current_v_B + dv_B/m_B;
next_v_C = current_v_C + dv_C/m_C;
next_v_D = current_v_D + dv_D/m_D;

next_x_A = current_x_A + dt*next_v_A;
next_x_B = current_x_B + dt*next_v_B;
next_x_C = current_x_C + dt*next_v_C;
next_x_D = current_x_D + dt*next_v_D;

%%
for i=1:200
    current_v_A = next_v_A;
    current_v_B = next_v_B;
    current_v_C = next_v_C;
    current_v_D = next_v_D;
    
    current_x_A = next_x_A;
    current_x_B = next_x_B;
    current_x_C = next_x_C;
    current_x_D = next_x_D;
        
    F_gravity_A = m_A * g;
    F_gravity_B = m_B * g;
    F_gravity_C = m_C * g;
    F_gravity_D = m_D * g;
    
    F_damp_A = -C*current_v_A;
    F_damp_B = -C*current_v_B;
    F_damp_C = -C*current_v_C;
    F_damp_D = -C*current_v_D;
    
    F_inter_AB = F_inter(current_x_A,current_x_B,1,k);
    F_inter_AC = -k*(norm(current_x_A-current_x_C)-L_0)*(current_x_A-current_x_C)/norm(current_x_A-current_x_C);
    F_inter_AD = -k*(norm(current_x_A-current_x_D)-L_d)*(current_x_A-current_x_D)/norm(current_x_A-current_x_D);
    F_inter_BA = -k*(norm(current_x_B-current_x_A)-L_0)*(current_x_B-current_x_A)/norm(current_x_B-current_x_A);
    F_inter_BC = -k*(norm(current_x_B-current_x_C)-L_d)*(current_x_B-current_x_C)/norm(current_x_B-current_x_C);
    F_inter_BD = -k*(norm(current_x_B-current_x_D)-L_0)*(current_x_B-current_x_D)/norm(current_x_B-current_x_D);
    F_inter_CA = -k*(norm(current_x_C-current_x_A)-L_0)*(current_x_C-current_x_A)/norm(current_x_C-current_x_A);
    F_inter_CB = -k*(norm(current_x_C-current_x_B)-L_d)*(current_x_C-current_x_B)/norm(current_x_C-current_x_B);
    F_inter_CD = -k*(norm(current_x_C-current_x_D)-L_0)*(current_x_C-current_x_D)/norm(current_x_C-current_x_D);
    F_inter_DA = -k*(norm(current_x_D-current_x_A)-L_d)*(current_x_D-current_x_A)/norm(current_x_D-current_x_A);
    F_inter_DB = -k*(norm(current_x_D-current_x_B)-L_0)*(current_x_D-current_x_B)/norm(current_x_D-current_x_B);
    F_inter_DC = -k*(norm(current_x_D-current_x_C)-L_0)*(current_x_D-current_x_C)/norm(current_x_D-current_x_C);
    
    F_A = F_gravity_A + F_damp_A + F_inter_AB+F_inter_AC+F_inter_AD;
    F_B = F_gravity_B + F_damp_B + F_inter_BA+F_inter_BD+F_inter_BC;
    F_C = F_gravity_C + F_damp_C + F_inter_CA+F_inter_CD+F_inter_CB;
    F_D = F_gravity_D + F_damp_D + F_inter_DC+F_inter_DB+F_inter_DA;
    
    next_v_A = current_v_A+F_A/m_A;
    next_v_B = current_v_B+F_B/m_B;
    next_v_C = current_v_C+F_C/m_C;
    next_v_D = current_v_D+F_D/m_D;
    
    
    next_x_A = current_x_A + dt*next_v_A;
    next_x_B = current_x_B + dt*next_v_B;
    next_x_C = current_x_C + dt*next_v_C;
    next_x_D = current_x_D + dt*next_v_D;
    
    if next_x_A(2,1) <= 0
            next_v_A(2,1) =  next_v_A(2,1)*(-0.8);
            next_x_A(2,1) =  next_x_A(2,1)*(-0.8);
    end 

    if next_x_B(2,1) <= 0
            next_v_B(2,1) =  next_v_B(2,1)*(-0.8);
            next_x_B(2,1) =  next_x_B(2,1)*(-0.8);
    end 
    
    if next_x_C(2,1) <= 0
            next_v_C(2,1) =  next_v_C(2,1)*(-0.8);
            next_x_C(2,1) =  next_x_C(2,1)*(-0.8);
    end 
    
    if next_x_D(2,1) <= 0
            next_v_D(2,1) =  next_v_D(2,1)*(-0.8);
            next_x_D(2,1) =  next_x_D(2,1)*(-0.8);
    end 
    
    scatter(next_x_A(1,1),next_x_A(2,1),'.b');
    hold on
    scatter(next_x_B(1,1),next_x_B(2,1),'.r');
    scatter(next_x_C(1,1),next_x_C(2,1),'.m');
    scatter(next_x_D(1,1),next_x_D(2,1),'.k');
    plot([next_x_A(1,1),next_x_B(1,1)],[next_x_A(2,1),next_x_B(2,1)]);
    plot([next_x_A(1,1),next_x_C(1,1)],[next_x_A(2,1),next_x_C(2,1)]);
    plot([next_x_A(1,1),next_x_D(1,1)],[next_x_A(2,1),next_x_D(2,1)]);
    plot([next_x_B(1,1),next_x_C(1,1)],[next_x_B(2,1),next_x_C(2,1)]);
    plot([next_x_B(1,1),next_x_D(1,1)],[next_x_B(2,1),next_x_D(2,1)]);
    plot([next_x_C(1,1),next_x_D(1,1)],[next_x_C(2,1),next_x_D(2,1)]);
    hold off
    xlim([-1 4]);
    ylim([0 50]);
    movieVector(i) = getframe;
end
%%
myWriter = VideoWriter('move4');
myWriter.FrameRate = 10;

open(myWriter);
writeVideo(myWriter, movieVector);
close(myWriter);