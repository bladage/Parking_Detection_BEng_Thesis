import pandas as pd

df = pd.read_csv('0_measurements.csv')
df_new = df.groupby("time").sum()
df_new.to_csv('0_measurements_merged.csv')

# data = {
  # 'co2': [95, 90, 99, 104, 105, 94, 99, 104],
  # 'model': ['Citigo', 'Fabia', 'Fiesta', 'Rapid', 'Focus', 'Mondeo', 'Octavia', 'B-Max'],
  # 'car': ['Skoda', 'Skoda', 'Ford', 'Skoda', 'Ford', 'Ford', 'Skoda', 'Ford']
# }
# df = pd.DataFrame(data)
# df_new = df.groupby("car").sum()

# df_new.to_csv('out.csv')
