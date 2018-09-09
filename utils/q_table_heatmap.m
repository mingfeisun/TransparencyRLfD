authors = {'annabelle', 'chen', 'cris', 'dong', ...
            'feng', 'jerry', 'jiahui', 'jie', ...
            'joey', 'li', 'peng', 'rex', ...
            'shen', 'shu', 'wang', 'xiang', ...
            'xin', 'xue', 'zhou', 'ziming'};

folder = 'data/q_table';
modes = {'mode1', 'mode2', 'mode3', 'mode4'};

data = zeros(10, 10, 4);

for idx_mode=1:length(modes)
    for idx_author = 1:length(authors)
        filename = sprintf('%s/%s-q_table_value.txt', modes{idx_mode}, authors{idx_author});
        filename_path = fullfile(folder, filename);
        tmp_raw_data = importdata(filename_path);
        tmp_data = max(tmp_raw_data.data, [], 2);
        for i =1:10
            for j =1:10
                data(i, j, idx_mode) = data(i, j, idx_mode) + tmp_data((i-1)*10 + j);
            end
        end
    end
    data(:, :, idx_mode) = data(:, :, idx_mode) / 20.0;
end

for i = 1:4
    title_str = sprintf('Mode %d', i);
    subplot(1, 4, i);
    tmp_data = data(:, :, i);

    picture = imread('data/heatmap-table-cropped.png');
    [height,width,depth] = size(picture);

    X = [];
    Y = [];
    val = [];
    idx_tmp = 1;

    for i = 1:10
        for j = 1:10
            Y(idx_tmp) = (i-0.5)/10.0 * height;
            X(idx_tmp) = (j-0.5)/10.0 * width;
            val(idx_tmp) = tmp_data(i, j);
            idx_tmp = idx_tmp + 1;
        end
    end

    OverlayImage=[];
    F = scatteredInterpolant(Y', X', val','natural');
    % F = scatteredInterpolant(Y', X', val','linear');
    for i = 1:height-1
       for j = 1:width-1
              OverlayImage(i,j) = F(i,j);
       end
    end
    alpha = (~isnan(OverlayImage))*0.5;

    imshow(picture);
    hold on
    OverlayImage = imshow( OverlayImage );
    % Set the color limits to be relative to the data values
    % caxis auto;
    caxis([-10 25]);  
    colormap( OverlayImage.Parent, jet );
    colorbar( OverlayImage.Parent );
    % Set the AlphaData to be the transparency matrix created earlier
    set( OverlayImage, 'AlphaData', alpha );
end