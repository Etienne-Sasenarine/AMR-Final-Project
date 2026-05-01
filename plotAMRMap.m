function plotAMRMap(filename)
% plotAMRMap - Plots AMR simulation map from file
%
% Usage:
%   plotAMRMap('map.txt')

    figure; hold on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)');
    title('AMR Map');
    grid on;

    fid = fopen(filename, 'r');
    if fid == -1
        error('Could not open file');
    end

    while ~feof(fid)
        lineStr = strtrim(fgetl(fid));

        % Skip empty or comment lines
        if isempty(lineStr) || startsWith(lineStr, '%')
            continue;
        end

        tokens = strsplit(lineStr);
        type = tokens{1};

        switch type
            case 'wall'
                % wall x1 y1 x2 y2
                vals = str2double(tokens(2:5));
                plot([vals(1), vals(3)], [vals(2), vals(4)], ...
                     'k-', 'LineWidth', 2);

            case 'line'
                % line x1 y1 x2 y2
                vals = str2double(tokens(2:5));
                plot([vals(1), vals(3)], [vals(2), vals(4)], ...
                     'b--', 'LineWidth', 1.2);

            case 'beacon'
                % beacon x y [r g b] ID
                x = str2double(tokens{2});
                y = str2double(tokens{3});

                % Extract RGB (tokens like "[0.0", "0.0", "0.0]")
                rgbStr = tokens(4:6);
                rgbStr{1} = erase(rgbStr{1}, '[');
                rgbStr{3} = erase(rgbStr{3}, ']');
                rgb = str2double(rgbStr);

                id = tokens{7};

                plot(x, y, 'o', ...
                     'MarkerFaceColor', rgb, ...
                     'MarkerEdgeColor', 'k', ...
                     'MarkerSize', 6);

                text(x + 0.05, y + 0.05, id, 'FontSize', 8);

            case 'virtwall'
                % Optional: draw ray for virtual wall
                x = str2double(tokens{2});
                y = str2double(tokens{3});
                theta = str2double(tokens{4});

                L = 1.0; % length of virtual wall visualization
                x2 = x + L*cos(theta);
                y2 = y + L*sin(theta);

                plot([x x2], [y y2], 'r:', 'LineWidth', 1.5);
        end
    end

    fclose(fid);

    legend({'Wall', 'Line', 'Beacon'}, 'Location', 'bestoutside');
end