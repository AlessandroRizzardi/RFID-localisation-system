
%phase_history(k,1) = 0;

if tag_found_flag == false
   if robots(i).inTagRange(targets(i,:)) <= 0.3
    targets(i,:) = generateRandomPointInCircle([0,0],(x_range(2) - x_range(1))/2);
   end

   target_point = targets(i,:);
   [v,omega] = greedy_controller(Kp_v1, Kp_w1, target_point(1),target_point(2), robots(i).x_est);

elseif tag_found_flag == true

    targets(i,:) = tag_found_position;

    target_point = targets(i,:);
    [v,omega] = greedy_controller(Kp_v1, Kp_w1, target_point(1),target_point(2), robots(i).x_est);  

end

x_next = robots(i).dynamics(v,omega);
robots(i).dynamics_history{k,1} = x_next;

odometry_estimation = robots(i).odometry_step(v,omega);
robots(i).odometry_history{k,1} = robots(i).x_est;


