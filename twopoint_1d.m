%% constant delacre

clear all;

m_A = 1;
m_B = 1;
g = [0;-1];
C = 0.03;
dt = 0.1;
k = 1;  
%% memory allocate

x_A = [0;10];
x_B = [1;11];
v_A = [0;0];
v_B = [0;0];
L_0 = 1;
%%
F_A = [0;0];
F_B = [0;0];
for i=1:50
    
    v_A = v_A+F_A/m_A/2;
    v_B = v_B+F_B/m_B/2;
    x_A = x_A+dt*v_A;
    x_B = x_B+dt*v_B;

    if x_A(2,1) <= 0
         v_A(2,1) =  v_A(2,1)*(-1);
         x_A(2,1) =  x_A(2,1)*(-1);
    end
    
    if x_B(2,1) <= 0
         v_B(2,1) =  v_B(2,1)*(-1);
         x_B(2,1) =  x_B(2,1)*(-1);
    end 

    F_gravity_A = m_A * g;
    F_gravity_B = m_B * g;
    F_damp_A = -C*v_A;
    F_damp_B = -C*v_B;
    F_inter_A = -k*(norm(x_A-x_B)-L_0)*(x_A-x_B)/norm(x_A-x_B);
    F_inter_B = -k*(norm(x_A-x_B)-L_0)*(x_B-x_A)/norm(x_A-x_B);
    F_A = F_gravity_A + F_damp_A + F_inter_A;
    F_B = F_gravity_B + F_damp_B + F_inter_B;
    
    v_A = v_A+F_A/m_A/2;
    v_B = v_B+F_B/m_B/2;
    
    
    plot(x_A(1,1),x_A(2,1),'.b');
    hold on
    plot(x_B(1,1),x_B(2,1),'.r');
    hold off
    ylim([0 10]);
    xlim([0 10]);
    movieVector(i) = getframe;
end

%%
myWriter = VideoWriter('move');
myWriter.FrameRate = 10;

open(myWriter);
writeVideo(myWriter, movieVector);
close(myWriter);