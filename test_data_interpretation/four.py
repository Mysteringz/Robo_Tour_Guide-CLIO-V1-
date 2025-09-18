import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import statsmodels.api as sm
from statsmodels.formula.api import ols
from statsmodels.stats.multicomp import pairwise_tukeyhsd

# -------------------------------------
# Utility functions
# -------------------------------------

def cronbach_alpha(itemscores):
    itemscores = np.asarray(itemscores, dtype=float)
    itemvars = itemscores.var(axis=0, ddof=1)
    tscores = itemscores.sum(axis=1)
    nitems = itemscores.shape[1]
    return nitems / (nitems - 1) * (1 - itemvars.sum() / tscores.var(ddof=1))

def generate_synthetic_data(n_per_group=30):
    np.random.seed(20250902)
    groups = ["Random", "Human", "AI"]
    subscales = {
        "Anthropomorphism": 5,
        "Animacy": 5,
        "Likeability": 5,
        "Perceived_Intelligence": 5,
        "Perceived_Safety": 5
    }

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
    df = pd.DataFrame(data)

    # compute subscale means
    for scale, n_items in subscales.items():
        cols = [f"{scale}_Q{i}" for i in range(1, n_items + 1)]
        df[scale] = df[cols].mean(axis=1)

    return df, subscales

def detect_subscales_from_csv(df):
    subscales = {}
    for col in df.columns:
        if '_Q' in col:
            scale_name = col.split('_Q')[0]
            subscales.setdefault(scale_name, 0)
            subscales[scale_name] += 1
    return subscales

# -------------------------------------
# GUI App
# -------------------------------------

class AnalysisApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Godspeed Questionnaire Analysis")
        self.df = None
        self.subscales = None
        self.current_fig = None

        # Buttons
        ttk.Button(root, text="Load CSV", command=self.load_csv).pack(pady=5)
        ttk.Button(root, text="Load Random Data", command=self.load_random).pack(pady=5)
        ttk.Button(root, text="Run Analysis", command=self.run_analysis).pack(pady=5)

        # Notebook for tabs
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True)

        # Graph tab
        self.graph_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.graph_tab, text="Graph")

        # Dropdown for graph selection
        self.scale_var = tk.StringVar()
        self.scale_menu = ttk.Combobox(self.graph_tab, textvariable=self.scale_var, state="readonly")
        self.scale_menu.pack(pady=5)
        self.scale_menu.bind("<<ComboboxSelected>>", self.show_graph)

        # Figure frame
        self.fig_frame = ttk.Frame(self.graph_tab)
        self.fig_frame.pack(fill="both", expand=True)

        ttk.Button(self.graph_tab, text="Save Current Graph", command=self.save_graph).pack(pady=5)

        # Stats Table tab
        self.table_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.table_tab, text="Stats Table")

        # Treeview for results
        self.tree = ttk.Treeview(self.table_tab)
        self.tree.pack(fill="both", expand=True)
        self.tree_scroll = ttk.Scrollbar(self.table_tab, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=self.tree_scroll.set)
        self.tree_scroll.pack(side="right", fill="y")

    # -----------------------------
    # Data loading
    # -----------------------------
    def load_csv(self):
        file_path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv"), ("All Files", "*.*")])
        if file_path:
            try:
                self.df = pd.read_csv(file_path)
                self.subscales = detect_subscales_from_csv(self.df)
                for scale, n_items in self.subscales.items():
                    cols = [f"{scale}_Q{i}" for i in range(1, n_items + 1)]
                    self.df[scale] = self.df[cols].mean(axis=1)
                messagebox.showinfo("CSV Loaded", f"Successfully loaded {file_path}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load CSV:\n{e}")

    def load_random(self):
        self.df, self.subscales = generate_synthetic_data()
        messagebox.showinfo("Info", "Random dataset generated!")

    # -----------------------------
    # Analysis
    # -----------------------------
    def run_analysis(self):
        if self.df is None or self.subscales is None:
            messagebox.showerror("Error", "Please load data first!")
            return

        # Populate dropdown menu
        self.scale_menu["values"] = list(self.subscales.keys())
        if not self.scale_var.get() and self.scale_menu["values"]:
            self.scale_menu.current(0)
            self.show_graph()

        # Prepare stats table
        self.tree.delete(*self.tree.get_children())

        # Columns: group1, group2, meandiff, p-adj, lower, upper, reject
        columns = ["Scale", "group1", "group2", "meandiff", "p-adj", "lower", "upper", "reject"]
        self.tree["columns"] = columns
        self.tree["show"] = "headings"
        for col in columns:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=100, anchor="center")

        # Run ANOVA + Tukey
        for scale in self.subscales.keys():
            try:
                model = ols(f"{scale} ~ C(Group)", data=self.df).fit()
                anova_table = sm.stats.anova_lm(model, typ=2)
                # Add ANOVA summary row
                self.tree.insert("", "end", values=[f"{scale} ANOVA", "-", "-", "-", "-", "-", "-", "-"])
                for idx, row in anova_table.iterrows():
                    self.tree.insert("", "end", values=[scale, idx, "", row["sum_sq"], row["df"], row["F"], row["PR(>F)"], ""])
                # Tukey
                tukey = pairwise_tukeyhsd(self.df[scale], self.df["Group"])
                # Add Tukey header
                self.tree.insert("", "end", values=[f"{scale} Tukey HSD", "-", "-", "-", "-", "-", "-", "-"])
                for i in range(len(tukey.summary())):
                    if i == 0 or i == 1:
                        continue  # skip header lines
                    line = tukey.summary()[i]
                    parts = line.split()
                    if len(parts) >= 7:
                        self.tree.insert("", "end", values=[scale] + parts[:7])
            except Exception as e:
                self.tree.insert("", "end", values=[scale, "Error", str(e), "-", "-", "-", "-", "-"])

    # -----------------------------
    # Graph display
    # -----------------------------
    def show_graph(self, event=None):
        if self.df is None or self.subscales is None:
            return
        scale = self.scale_var.get()
        if not scale:
            return

        for widget in self.fig_frame.winfo_children():
            widget.destroy()

        np.random.seed(42)
        fig, ax = plt.subplots(figsize=(6, 4))
        sns.boxplot(x="Group", y=scale, data=self.df, palette="Set2", ax=ax)
        sns.stripplot(x="Group", y=scale, data=self.df, color="black", alpha=0.6, ax=ax, jitter=True, dodge=True)
        ax.set_title(f"{scale} by Group")
        ax.set_ylabel("Mean Score (1 = agreeable, 5 = less agreeable)")

        self.current_fig = fig
        canvas = FigureCanvasTkAgg(fig, master=self.fig_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)

    # -----------------------------
    # Save current graph
    # -----------------------------
    def save_graph(self):
        if self.current_fig is None:
            messagebox.showerror("Error", "No graph to save!")
            return
        file_path = filedialog.asksaveasfilename(defaultextension=".png",
                                                 filetypes=[("PNG files", "*.png"), ("All Files", "*.*")])
        if file_path:
            self.current_fig.savefig(file_path, dpi=300)
            messagebox.showinfo("Saved", f"Graph saved to {file_path}")

# -------------------------------------
# Run app
# -------------------------------------
if __name__ == "__main__":
    root = tk.Tk()
    app = AnalysisApp(root)
    root.mainloop()
