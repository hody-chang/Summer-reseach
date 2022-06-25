function F_inter = F_inter(x_1,x_2,L_0,k)

F_inter = -k*(norm(x_1-x_2)-L_0)*(x_1-x_2)/norm(x_1-x_2);

end

