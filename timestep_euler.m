function x1 = timestep_euler(Ts,x0, u0)
    x1 = x0 + Ts * quadcopter_ode(Ts,x0,u0);
end
