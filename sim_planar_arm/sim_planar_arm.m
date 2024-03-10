
function sim_planar_arm (theta1,theta2,l1,l2,x_end,y_end)
    dim = length(theta2); %lunghezza del vettore theta2 per il ciclo for

    for i=1:dim
        x0 = 0;
        y0 = 0;
        
        %posizione del secondo giunto
        x1 = round(l1*cosd(theta1(i)),3);
        y1 = round(l1*sind(theta1(i)),3);
        
        %posizone dell'end-effector
        x2 = round((x1 + l2*cosd(theta1(i)+theta2(i))),3);
        y2 = round((y1 + l2*sind(theta1(i)+theta2(i))),3);
        
        %plot
        clf
        %plot dei due link
        plot([x0 x1],[y0 y1], [x1 x2],[y1 y2],LineWidth=2,Color="black")
        hold on
        % plot dei due giunti e dell'effector
        plot([x0 x1 x2],[y0 y1 y2],'.k','MarkerSize',15)
        plot([x0 x1],[y0 y1],'ok','MarkerSize',10)
        %plot della traiettoria dell'end-effector
        plot([x0 x_end],[y0 y_end],'--r')
        axis([-10 10 -10 10])
        grid on
        refreshdata
        
        drawnow
        pause(1.5)
        
    end
end

