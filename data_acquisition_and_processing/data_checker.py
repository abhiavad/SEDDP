import os
import glob
import pandas as pd
import matplotlib.pyplot as plt


def load_and_plot(csv_files):
    if not csv_files:
        print("No CSV files found in the specified directory.")
        return

    plt.figure(figsize=(14, 6))

    # Plot Roll vs Time
    plt.subplot(1, 2, 1)
    for file in csv_files:
        try:
            data = pd.read_csv(file, header=None)
            time = pd.to_numeric(data.iloc[:, 774], errors='coerce')
            roll = pd.to_numeric(data.iloc[:, 770], errors='coerce')
            valid = time.notna() & roll.notna()
            plt.plot(time[valid], roll[valid], label=os.path.basename(file))
        except Exception as e:
            print(f"Error processing file {file}: {e}")
    plt.title('Pitch vs Time')
    plt.xlabel('Time')
    plt.ylabel('Pitch Angle')
    plt.legend(fontsize="x-small")

    # Plot Pitch vs Time
    plt.subplot(1, 2, 2)
    for file in csv_files:
        try:
            data = pd.read_csv(file, header=None)
            time = pd.to_numeric(data.iloc[:, 774], errors='coerce')
            pitch = pd.to_numeric(data.iloc[:, 771], errors='coerce')
            valid = time.notna() & pitch.notna()
            plt.plot(time[valid], pitch[valid], label=os.path.basename(file))
        except Exception as e:
            print(f"Error processing file {file}: {e}")
    plt.title('Roll vs Time')
    plt.xlabel('Time')
    plt.ylabel('Roll Angle')
    plt.legend(fontsize="x-small")

    plt.tight_layout()
    plt.show()

def main():
    folder_path = input("Enter the path to the CSV files directory: ").strip()
    if not os.path.isdir(folder_path):
        print("Invalid folder path.")
        return

    csv_files = sorted(glob.glob(os.path.join(folder_path, "*.csv")))
    load_and_plot(csv_files)

if __name__ == "__main__":
    main()