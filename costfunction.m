function cost = costfunction(y, H)

   [H,Ts,id_u1,id_u2,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta,id_f1, id_f2] = drone_info;
   [mass,inertia_moment,arm_moment,gravitational_acceleration] = parameters;
    %final positions
    x_final=10;
    z_final=10;

%     common_final = mass*gravitational_acceleration;
common_final=0;
    % Unpacking
   
     
    x = y(id_x);
    z = y(id_z);
    theta = y(id_theta);
    x_velocity = y(id_dotx);
    z_velocity = y(id_dotz);
    angular_velocity = y(id_dottheta);
    force1 = y(id_f1);
    force2 = y(id_f2);
    
    mode_diff = y(id_u1);
    mode_common = y(id_u2);
   

    cost =  sum((x(:)-x_final).^2+20*(z(:)-z_final).^2 +50*(theta(:)).^2 + (x_velocity(:)).^2+ 5*(z_velocity(:)).^2+20*(angular_velocity(:)).^2 + 50*(mode_diff(:)).^2+50*(mode_common(:)-common_final).^2+5*(force1(:)).^2+20*(force2(:)).^2);

end