import os
import scipy.io
import pandas as pd

def matlab_to_csv_folder(mat_folder, csv_folder):
    if not os.path.exists(csv_folder):
        os.makedirs(csv_folder)
    for filename in os.listdir(mat_folder):
        if filename.endswith('.mat'):
            mat_path = os.path.join(mat_folder, filename)
            data = scipy.io.loadmat(mat_path)
            # Remove MATLAB metadata keys
            data = {k: v for k, v in data.items() if not k.startswith('__')}
            for key, value in data.items():
                # Only save arrays that can be converted to DataFrame
                try:
                    df = pd.DataFrame(value)
                    csv_path = os.path.join(csv_folder, f"{os.path.splitext(filename)[0]}_{key}.csv")
                    df.to_csv(csv_path, index=False)
                except Exception as e:
                    print(f"Skipping {key} in {filename}: {e}")