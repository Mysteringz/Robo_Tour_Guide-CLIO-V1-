import tkinter as tk
from tkinter import filedialog, scrolledtext, messagebox
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import seaborn as sns
from scipy import stats
import statsmodels.api as sm
from statsmodels.formula.api import ols
from statsmodels.stats.multicomp import pairwise_tukeyhsd

# -----------------------------
# Config
# -----------------------------
np.random.seed(20250902)
subscales = {
    "Anthropomorphism": 5,
    "Animacy": 5,
    "Likeability": 5,
    "Perceived_Intelligence": 5,
    "Perceived_Safety": 5
}
groups = ["Random", "Human", "AI"]

# -----------------------------
# Helper functions
# -----------------------------
def cronbach_alpha(itemscores):
    itemscores = np.asarray(itemscores, dtype=float)
    itemvars = itemscores.var(axis=0, ddof=1)
    tscores = itemscores.sum(axis=1)
    nitems = itemscores.shape[1]
    return nitems / (nitems - 1) * (1 - itemvars.sum() / tscores.var(ddof=1))

def generate_synthetic_data(n_per_group=30):
    data = []
    for g in groups:
        for pid in range(n_per_group):
            participant = {"Group": g, "ID": f"{g}_{pid+1}"}
            for scale, n_items in subscales.items():
                if g == "Random":
                    mu = 3.2
                elif g == "Human":
                    mu = 2.6
                else:
                    mu = 2.4
                responses = np.clip(np.round(np.random.normal(mu, 0.7, n_items)), 1, 5)
                for i, r in enumerate(responses, 1):
                    participant[f"{scale}_Q{i}"] = r
            data.append(participant)
    return pd.DataFrame(data)

# -----------------------------
# GUI logic
# -----------------------------
class AnalysisApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Godspeed Questionnaire Analysis")
        self.df = None

        # Buttons
        tk.Button(root, text="Load CSV", command=self.load_csv).pack(pady=5)
        tk.Button(root, text="Generate Random Data", command=self.load_random).pack(pady=5)
        tk.Button(root, text="Run Analysis", command=self.run_analysis).pack(pady=5)

        # Text box for results
        self.text_box = scrolledtext.ScrolledText(root, width=100, height=25)
        self.text_box.pack(pady=5)

        # Frame for plots
        self.plot_frame = tk.Frame(root)
        self.plot_frame.pack(fill="both", expand=True)

    def load_csv(self):
        file_path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
        if file_path:
            self.df = pd.read_csv(file_path)
            messagebox.showinfo("Data Loaded", f"Loaded {file_path}")

    def load_random(self):
        self.df = generate_synthetic_data()
        messagebox.showinfo("Random Data", "Generated synthetic dataset (n=90).")

    def run_analysis(self):
        if self.df is None:
            messagebox.showwarning("No Data", "Please load or generate data first.")
            return

        self.text_box.delete("1.0", tk.END)
        df = self.df.copy()

        # Compute Cronbach’s alpha and subscale scores
        self.text_box.insert(tk.END, "Cronbach’s alpha per subscale:\n")
        for scale, n_items in subscales.items():
            cols = [f"{scale}_Q{i}" for i in range(1, n_items + 1)]
            alpha = cronbach_alpha(df[cols])
            self.text_box.insert(tk.END, f"  {scale}: {alpha:.2f}\n")
            df[scale] = df[cols].mean(axis=1)

        # Clear old plots
        for widget in self.plot_frame.winfo_children():
            widget.destroy()

        # Run ANOVA + Tukey
        for scale in subscales.keys():
            self.text_box.insert(tk.END, f"\n--- {scale} ---\n")
            model = ols(f"{scale} ~ C(Group)", data=df).fit()
            anova_table = sm.stats.anova_lm(model, typ=2)
            self.text_box.insert(tk.END, f"{anova_table}\n")

            # Tukey post-hoc
            tukey = pairwise_tukeyhsd(df[scale], df["Group"])
            self.text_box.insert(tk.END, f"{tukey}\n")

            # Visualization
            fig, ax = plt.subplots(figsize=(5, 4))
            sns.boxplot(x="Group", y=scale, data=df, palette="Set2", ax=ax)
            sns.stripplot(x="Group", y=scale, data=df, color="black", alpha=0.6, ax=ax)
            ax.set_title(f"{scale} by Group")
            ax.set_ylabel("Mean Score (1 = agreeable, 5 = less agreeable)")
            canvas = FigureCanvasTkAgg(fig, master=self.plot_frame)
            canvas.draw()
            canvas.get_tk_widget().pack(side=tk.LEFT, padx=10, pady=10)

# -----------------------------
# Run GUI
# -----------------------------
if __name__ == "__main__":
    root = tk.Tk()
    app = AnalysisApp(root)
    root.mainloop()
