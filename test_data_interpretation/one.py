import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
import statsmodels.api as sm
from statsmodels.formula.api import ols
from statsmodels.stats.multicomp import pairwise_tukeyhsd

# -----------------------------
# 1. Generate synthetic dataset
# -----------------------------
np.random.seed(20250902)

n_per_group = 30
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
            # group-specific means (to simulate differences)
            if g == "Random":
                mu = 3.2
            elif g == "Human":
                mu = 2.6
            else:  # AI
                mu = 2.4
            # generate Likert responses (1–5)
            responses = np.clip(np.round(np.random.normal(mu, 0.7, n_items)), 1, 5)
            for i, r in enumerate(responses, 1):
                participant[f"{scale}_Q{i}"] = r
        data.append(participant)

df = pd.DataFrame(data)

# -----------------------------
# 2. Reliability (Cronbach’s α)
# -----------------------------
def cronbach_alpha(itemscores):
    itemscores = np.asarray(itemscores, dtype=float)
    itemvars = itemscores.var(axis=0, ddof=1)
    tscores = itemscores.sum(axis=1)
    nitems = itemscores.shape[1]
    return nitems / (nitems - 1) * (1 - itemvars.sum() / tscores.var(ddof=1))

alpha_results = {}
for scale, n_items in subscales.items():
    cols = [f"{scale}_Q{i}" for i in range(1, n_items + 1)]
    alpha = cronbach_alpha(df[cols])
    alpha_results[scale] = alpha

print("Cronbach’s alpha per subscale:")
for scale, alpha in alpha_results.items():
    print(f"  {scale}: {alpha:.2f}")

# -----------------------------
# 3. Subscale scores (averages)
# -----------------------------
for scale, n_items in subscales.items():
    cols = [f"{scale}_Q{i}" for i in range(1, n_items + 1)]
    df[scale] = df[cols].mean(axis=1)

# -----------------------------
# 4. ANOVA + Post-hoc
# -----------------------------
results = {}
for scale in subscales.keys():
    model = ols(f"{scale} ~ C(Group)", data=df).fit()
    anova_table = sm.stats.anova_lm(model, typ=2)
    results[scale] = anova_table

    # Welch’s ANOVA alternative
    welch = stats.f_oneway(
        df[df.Group == "Random"][scale],
        df[df.Group == "Human"][scale],
        df[df.Group == "AI"][scale]
    )

    print("\n---", scale, "---")
    print("ANOVA table:")
    print(anova_table)
    print("Welch F-test:", welch)

    # Tukey post-hoc
    tukey = pairwise_tukeyhsd(df[scale], df["Group"])
    print(tukey)

# -----------------------------
# 5. Visualization
# -----------------------------
sns.set(style="whitegrid")
for scale in subscales.keys():
    plt.figure(figsize=(6, 4))
    sns.boxplot(x="Group", y=scale, data=df, palette="Set2")
    sns.stripplot(x="Group", y=scale, data=df, color="black", alpha=0.6)
    plt.title(f"{scale} by Group")
    plt.ylabel("Mean Score (1 = agreeable, 5 = less agreeable)")
    plt.show()
