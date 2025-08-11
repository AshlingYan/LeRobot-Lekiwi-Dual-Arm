import pandas as pd

# 读取 Parquet 文件
df = pd.read_parquet("/home/robotlab/svla_so101_pickplace/data/chunk-000/file-000.parquet")

# 显示前 5 行
print(df.head())

# 显示所有列名
print(df.columns)

# 显示数据统计信息
print(df.describe())
