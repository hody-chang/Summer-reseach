%% declare
clear all;

m = 1;
g = [0;-9.81];
C = 0.03;
dt = 0.01;
k = 100;
nstep = 500;
n = 5;
%% memory
x = zeros(2,n*n);
v = zeros(2,n*n);

L_0 = 1;
L_d = sqrt(2*L_0^2);


F_in = zeros(2,n*n);
F_total = zeros(2,n*n);
%% gind setting
botton_left = [1;50];

for i= 1:n
    for j = 1:n
        x(:,(i-1)*n+j)= botton_left + [(j-1)*1;i*1];
    end
end
%%
for i=1:nstep

    
    v = v + dt/2*F_total/m;
    x = x + dt*v;
    
    F_total = zeros(2,n*n);
    F_in = zeros(2,n*n);
    
    for j=1:n*n
        if x(2,j) <= 0
            v(2,j) =  v(2,j)*(-1);
            x(2,j) =  x(2,j)*(-1);
        end 
    end
      
    for j=1:n %o-o
        for k=1:n-1
            F_in(:,(j-1)*n+k) = F_in(:,(j-1)*n+k)+F_inter(x(:,(j-1)*n+k),x(:,(j-1)*n+k+1),L_0,k);
            F_in(:,(j-1)*n+k+1) = F_in(:,(j-1)*n+k)+F_inter(x(:,(j-1)*n+k+1),x(:,(j-1)*n+k),L_0,k);
        end
    end
    
    for j=1:n-1 %o|o
        for k=1:n
            F_in(:,(j-1)*n+k) = F_in(:,(j-1)*n+k)+F_inter(x(:,(j-1)*n+k),x(:,(j)*n+k),L_0,k);
            F_in(:,(j)*n+k) = F_in(:,(j-1)*n+k)+F_inter(x(:,(j)*n+k),x(:,(j-1)*n+k),L_0,k);
        end
    end
    
    for j=1:n-1 %o/o
        for k=1:n-1
            F_in(:,(j-1)*n+k) = F_in(:,(j-1)*n+k)+F_inter(x(:,(j-1)*n+k),x(:,(j)*n+k+1),L_d,k);
            F_in(:,(j)*n+k+1) = F_in(:,(j-1)*n+k)+F_inter(x(:,(j)*n+k+1),x(:,(j-1)*n+k),L_d,k);
        end
    end
    
    for j=2:n %o\o
        for k=1:n-1
            F_in(:,(j-1)*n+k) = F_in(:,(j-1)*n+k)+F_inter(x(:,(j-1)*n+k),x(:,(j-2)*n+k+1),L_d,k);
            F_in(:,(j-2)*n+k+1) = F_in(:,(j-1)*n+k)+F_inter(x(:,(j-2)*n+k+1),x(:,(j-1)*n+k),L_d,k);
        end
    end
    
    for j= 1:n*n
            F_total(:,j) = m .* g -C.*v(:,j)+F_in(:,j);
    end
    
    F_total(:,21) = [0;0];
    F_total(:,25) = [0;0];
    
    
    v = v+ dt/2*F_total/m;
    
    scatter(x(1,1:5),x(2,1:5),'.r');
    hold on
    scatter(x(1,6:25),x(2,6:25),'.b');
    hold off
    
    xlim([0 5]);
    ylim([0 100]);
    movieVector(i) = getframe;
end
%%
myWriter = VideoWriter('moven');
myWriter.FrameRate = 10;

open(myWriter);
writeVideo(myWriter, movieVector);
close(myWriter);