function sim_planar_arm (theta1,theta2,l1,l2,pausa)
    dim = length(theta2); %length of angle vector
    
    x0 = 0;
    y0 = 0;
    x_end = round((l1*cosd(theta1(end)) + l2*cosd(theta1(end)+theta2(end))),3);
    y_end = round((l1*sind(theta1(end)) + l2*sind(theta1(end)+theta2(end))),3);

    for i=1:dim

        %position of first link
        x1 = round(l1*cosd(theta1(i)),3);
        y1 = round(l1*sind(theta1(i)),3);
        
        %position of second link
        x2 = round((x1 + l2*cosd(theta1(i)+theta2(i))),3);
        y2 = round((y1 + l2*sind(theta1(i)+theta2(i))),3);
        
        % plot
        clf
        % plot dei due link
        plot([x0 x1],[y0 y1], [x1 x2],[y1 y2],LineWidth=2,Color="black")
        hold on, grid on, axis([-10 10 -10 10])
        % plot dei due giunti e dell'effector
        plot([x0 x1 x2],[y0 y1 y2],'.k','MarkerSize',15)
        plot([x0 x1],[y0 y1],'ok','MarkerSize',10)
        % plot della traiettoria dell'end-effector
        plot([x0 x_end],[y0 y_end],'--g')
        
        pause(pausa)
        
    end
end
