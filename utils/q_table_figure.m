authors = {'annabelle', 'chen', 'cris', 'dong', ...
            'feng', 'jerry', 'jiahui', 'jie', ...
            'joey', 'li', 'peng', 'rex', ...
            'shen', 'shu', 'wang', 'xiang', ...
            'xin', 'xue', 'zhou', 'ziming'};

folder = './data/raw';
modes = {'mode1', 'mode2', 'mode3', 'mode4'};

data = zeros(54, 4);

for idx_mode=1:length(modes)
    count = 0;
    for idx_author = 1:length(authors)
        subfolder = sprintf('%s/%s/q_table', authors{idx_author}, modes{idx_mode});
        q_table_folder = fullfile(folder, subfolder);
        all_files = dir([q_table_folder '/*.txt']);
        if length(all_files) < 54
            continue
        end
        count = count + 1;
        for i = 1:54
            filename = sprintf('%s-q_table_value-%d.txt', authors{idx_author}, i);
            
            abs_path = fullfile(q_table_folder, filename);
            
            tmp_raw_data = importdata(abs_path);
            tmp_data = max(tmp_raw_data.data, [], 2);
            data(i, idx_mode) = data(i, idx_mode) + mean(tmp_data);
        end
    end
    disp(count);
    data(:, idx_mode) = data(:, idx_mode)/count;
end

demo_idx_array = mod(0:53, 6) == 0;
data = data(demo_idx_array, :);

figure();

for i = 1:4
    tmp_data = data(:, i);
    plot(tmp_data, '-*');
    hold on;
end
legend('Mode 1', 'Mode 2', 'Mode 3', 'Mode 4');