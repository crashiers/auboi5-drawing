from my_utils import *

filename = emotion + '_data_transfer_df_brush_5.csv'
data_transfer_df = pd.read_csv('data/' + filename) # 画什么

# data_transfer_df = data_transfer_df.iloc[257 : ]

indicator = np.ones(data_transfer_df.shape[0])
df = pd.DataFrame(columns={'num_physical_stroke','num_stroke','num_brush','start_point_x','start_point_y','end_point_x','end_point_y','theta_deg','theta_rad','r','g','b'})
while np.any(indicator):
    non_zero_index = np.where(indicator==1)
    # ic(non_zero_index)
    id = non_zero_index[0][0]
    # ic(id)
    indicator[id] = 0
    item = data_transfer_df.iloc[id]
    # ic(item)
    # ic((item['r'], item['g'], item['b']))

    cmyk = RGB2CMYK((item['r'], item['g'], item['b']))
    count = 0
    for i in range(1, len(non_zero_index[0])):
        item1 = data_transfer_df.iloc[non_zero_index[0][i]]
        cmyk1 = RGB2CMYK((item1['r'], item1['g'], item1['b']))
        # ic(cmyk, cmyk1)
        if judge_similar(cmyk, cmyk1):
            df = df.append(item1, ignore_index=True)
            indicator[non_zero_index[0][i]] = 0
            count += 1
    ic(count)

df.to_csv('data/' + f'After-grouping-{filename}', index=False)