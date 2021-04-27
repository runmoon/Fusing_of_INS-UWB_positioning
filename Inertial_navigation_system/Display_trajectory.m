% function Display_trajectory(X, Y)
% 
% text(X,Y,'°£');
% hold on;
% grid on;
% axis([-4 4 -4 4]);
% xlabel('X÷·');
% ylabel('Y÷·');
% drawnow
% 
% end


function Display_trajectory( coordinate )

figure(1);
plot3(coordinate(1), coordinate(2), coordinate(3), 'o');
hold on;
grid on;
axis([-1 1 -1 1 -1 1]);
xlabel('X÷·');
ylabel('Y÷·');
zlabel('Z÷·');
drawnow

end