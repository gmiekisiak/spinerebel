#!/usr/bin/env python3
"""
TARS-31 RIBBON PLOTTER — GUI
============================

Interactive viewer for the multi-panel SCEP ribbon figure built from
{day}_TARS31.csv files produced by the extractor.

Workflow:
  1. Browse to parent directory (containing *_TARS31.csv)
  2. Hit "Plot" -> multi-panel figure renders inline
  3. Toolbar gives zoom / pan / save
  4. "Save PNG" button writes ribbon_TARS31.png to parent dir
  5. "Blowout report" button shows which BIN files produced bad cycles
  6. Day-include checklist on left lets you toggle days off

Figure style:
  - blue dots = NZ Position (Valleys) = p0_r
  - red dots  = Excursion (Peaks)     = p3_r
  - smoothed rolling-median trend line per ribbon
  - title per panel: date / n / cohesion% / v / gap

NOTE ON EXTRACTOR VERSION
-------------------------
This viewer reads *_TARS31.csv, i.e. output of the TARS-31 extractor
generation. The frozen extractor of record for the accompanying manuscript is
TARS-60 v3. Before this is used to produce published figures, either point
CSV_SUFFIX at the TARS-60 v3 output naming, or confirm that the column
semantics of p0_r / p3_r are identical across generations. Derived biomarkers
are detector-version dependent and generations must not be mixed within one
comparative analysis. See CHANGELOG.md.

Author: Grzegorz Miekisiak, MD PhD / SpineRebel Technology
  (earlier drafts of this file carried a different surname; confirm the
   attribution you want before publication so that it matches the article)
"""

import os
import sys
import glob
from pathlib import Path
import numpy as np
import pandas as pd

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)


# ============================================================================
# CONFIG — match published figure style
# ============================================================================

CSV_SUFFIX  = '_TARS31'        # extractor output naming; see note above
VALLEY_COL  = 'p0_r'           # NZ Position (cm from hub at cycle start)
PEAK_COL    = 'p3_r'           # Excursion (cm from hub at apex)
COH_THRESH  = 3.0              # cm — gap < 3 cm = "cohesive cycle"
SMOOTH_WIN  = 21               # rolling median window for trend line
Y_TOP_CM    = 50.0             # plot ceiling
SCATTER_S   = 8                # scatter dot size
SCATTER_A   = 0.55             # scatter alpha

# Single source of truth for what counts as a blowout. Previously the
# per-day statistics used 60 cm while the blowout report used 50 cm, so the
# blowout count in panel titles disagreed with the report for any cycle
# falling between the two thresholds.
BLOWOUT_THRESH = 50.0          # cm

MIN_VALID_CYCLES = 5           # days with fewer valid cycles are skipped

# Color scheme
COL_VALLEY      = '#1f4ed8'    # deep blue
COL_PEAK        = '#d8261f'    # red
COL_VALLEY_LINE = '#5d80f0'    # lighter blue for trend
COL_PEAK_LINE   = '#f0726c'    # lighter red for trend


# ============================================================================
# CSV / STATS
# ============================================================================

def find_csvs(parent_dir):
    """Sorted list of {YYYY-MM-DD}{CSV_SUFFIX}.csv paths."""
    parent = Path(parent_dir)
    pattern = f"*{CSV_SUFFIX}.csv"
    csvs = sorted(glob.glob(str(parent / pattern)))
    if not csvs:
        csvs = sorted(glob.glob(str(parent / "**" / pattern), recursive=True))
    return csvs


def extract_date(csv_path):
    name = Path(csv_path).stem
    if name.endswith(CSV_SUFFIX):
        return name[:-len(CSV_SUFFIX)]
    return name


def load_day_csv(csv_path):
    try:
        df = pd.read_csv(csv_path)
        if len(df) == 0:
            return None
        if VALLEY_COL not in df.columns or PEAK_COL not in df.columns:
            return None
        return df
    except Exception:
        return None


def per_day_stats(df, blowout_thresh=BLOWOUT_THRESH):
    valleys = df[VALLEY_COL].values
    peaks = df[PEAK_COL].values
    gaps = peaks - valleys
    valid = ((peaks < blowout_thresh) & (valleys < blowout_thresh)
             & (gaps >= 0) & (gaps < blowout_thresh))
    if valid.sum() < MIN_VALID_CYCLES:
        return None
    v = valleys[valid]
    p = peaks[valid]
    g = gaps[valid]
    return {
        'n_total': len(df),
        'n_valid': int(valid.sum()),
        'n_blowout': int((~valid).sum()),
        'valley_median': float(np.median(v)),
        'peak_median':   float(np.median(p)),
        'gap_median':    float(np.median(g)),
        'cohesion_pct':  float(100.0 * (g < COH_THRESH).sum() / len(g)),
    }


def rolling_median(arr, win=SMOOTH_WIN):
    n = len(arr)
    if n < 3:
        return arr.copy()
    half = win // 2
    out = np.empty(n)
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out[i] = np.median(arr[lo:hi])
    return out


# ============================================================================
# PLOTTING
# ============================================================================

def plot_panel(ax, df, date_str, stats, show_legend=False,
               clip_blowouts=True):
    valleys = df[VALLEY_COL].values
    peaks = df[PEAK_COL].values
    n = len(df)
    x = np.arange(n)

    title = (f"{date_str}  •  n={n}  •  "
             f"coh<{COH_THRESH:.0f}cm={stats['cohesion_pct']:.0f}%  •  "
             f"v={stats['valley_median']:.1f}cm  "
             f"gap={stats['gap_median']:.1f}cm")
    if stats['n_blowout'] > 0:
        title += f"  •  ! {stats['n_blowout']} blowouts"
    ax.set_title(title, fontsize=10, fontweight='bold', loc='left')

    if clip_blowouts:
        v_plot = np.clip(valleys, 0, Y_TOP_CM)
        p_plot = np.clip(peaks, 0, Y_TOP_CM)
    else:
        v_plot, p_plot = valleys, peaks

    ax.scatter(x, v_plot, s=SCATTER_S, c=COL_VALLEY, alpha=SCATTER_A,
               label='NZ Position (Valleys)', edgecolors='none')
    ax.scatter(x, p_plot, s=SCATTER_S, c=COL_PEAK, alpha=SCATTER_A,
               label='Excursion (Peaks)', edgecolors='none')

    if n >= SMOOTH_WIN:
        ax.plot(x, rolling_median(v_plot), color=COL_VALLEY_LINE,
                lw=1.5, alpha=0.85)
        ax.plot(x, rolling_median(p_plot), color=COL_PEAK_LINE,
                lw=1.5, alpha=0.85)

    ax.set_xlabel('Cycle Count', fontsize=9)
    ax.set_ylabel('Magnitude (cm)', fontsize=9)
    ax.set_ylim(0, Y_TOP_CM)
    ax.set_xlim(-n * 0.02, n * 1.02)
    ax.grid(True, alpha=0.25, linestyle='--')

    if show_legend:
        ax.legend(loc='upper right', fontsize=8, framealpha=0.9)


def build_figure(panels, fig, ncols=3, title_suffix=None, clip_blowouts=True,
                 empty_message=None):
    """
    Render panels into the given figure.

    panels : list of (date_str, df, stats)
    """
    fig.clear()
    if not panels:
        ax = fig.add_subplot(111)
        ax.text(0.5, 0.5,
                empty_message or
                "No valid panels.\nLoad a directory and select days.",
                ha='center', va='center', fontsize=12, color='#888')
        ax.axis('off')
        return

    ncols = max(1, int(ncols))
    n = len(panels)
    nrows = int(np.ceil(n / ncols))
    base_title = f"SCEP Ribbons — {CSV_SUFFIX.lstrip('_')} cycles per day"
    if title_suffix:
        base_title = f"{base_title}  —  {title_suffix}"
    fig.suptitle(base_title, fontsize=12, fontweight='bold', y=0.995)

    for i, (date_str, df, stats) in enumerate(panels):
        ax = fig.add_subplot(nrows, ncols, i + 1)
        plot_panel(ax, df, date_str, stats,
                   show_legend=(i == 0),
                   clip_blowouts=clip_blowouts)

    fig.tight_layout(rect=[0, 0, 1, 0.97])


# ============================================================================
# GUI
# ============================================================================

class RibbonGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("TARS Ribbon Plotter")
        self.root.geometry("1500x950")
        self.root.minsize(1100, 700)

        self.parent_dir = tk.StringVar()
        self.csv_paths = []           # list[str]
        self.day_data = {}            # date_str -> (csv_path, df, stats)
        self.day_vars = {}            # date_str -> tk.BooleanVar
        self.skipped = []             # list[(filename, reason)]
        self.ncols_var = tk.IntVar(value=3)
        self.clip_var  = tk.BooleanVar(value=True)
        self.status    = tk.StringVar(
            value=f"Pick a parent directory containing *{CSV_SUFFIX}.csv files.")

        self._build_layout()

    # ---------- layout ----------

    def _build_layout(self):
        style = ttk.Style()
        try:
            style.theme_use('clam')
        except tk.TclError:
            pass

        # Top bar — folder + actions
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, text="Parent dir:").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.parent_dir, width=70).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        ttk.Button(top, text="Browse…", command=self._browse).pack(
            side=tk.LEFT, padx=4)
        ttk.Button(top, text="Load CSVs", command=self._load).pack(
            side=tk.LEFT, padx=4)
        ttk.Button(top, text="Plot", command=self._plot).pack(
            side=tk.LEFT, padx=4)
        ttk.Button(top, text="Save PNG", command=self._save).pack(
            side=tk.LEFT, padx=4)
        ttk.Button(top, text="Blowout report", command=self._blowout_report
                   ).pack(side=tk.LEFT, padx=4)

        # Body — left checklist, centre figure
        body = ttk.Frame(self.root)
        body.pack(fill=tk.BOTH, expand=True)

        # Left panel — day selector
        left = ttk.LabelFrame(body, text="Days to include", padding=6)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(8, 4), pady=(0, 8))

        # ncols + clip row
        opts = ttk.Frame(left)
        opts.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(opts, text="Cols:").pack(side=tk.LEFT)
        ttk.Spinbox(opts, from_=1, to=6, textvariable=self.ncols_var,
                    width=4, command=self._plot).pack(side=tk.LEFT, padx=(2, 8))
        ttk.Checkbutton(opts, text="Clip blowouts", variable=self.clip_var,
                        command=self._plot).pack(side=tk.LEFT)

        # All / None buttons
        btns = ttk.Frame(left)
        btns.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(btns, text="All", width=6,
                   command=lambda: self._toggle_all(True)).pack(
                       side=tk.LEFT, padx=2)
        ttk.Button(btns, text="None", width=6,
                   command=lambda: self._toggle_all(False)).pack(
                       side=tk.LEFT, padx=2)

        # Scrollable checklist
        self.list_canvas = tk.Canvas(left, width=240, highlightthickness=0)
        self.list_scroll = ttk.Scrollbar(left, orient='vertical',
                                         command=self.list_canvas.yview)
        self.list_canvas.configure(yscrollcommand=self.list_scroll.set)
        self.list_frame = ttk.Frame(self.list_canvas)
        self.list_canvas.create_window((0, 0), window=self.list_frame,
                                       anchor='nw')
        self.list_frame.bind('<Configure>',
                             lambda e: self.list_canvas.configure(
                                 scrollregion=self.list_canvas.bbox('all')))
        self.list_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.list_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        # Right panel — figure
        right = ttk.Frame(body)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True,
                   padx=(4, 8), pady=(0, 8))

        self.fig = Figure(figsize=(11, 8), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        toolbar_frame = ttk.Frame(right)
        toolbar_frame.pack(fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()

        # initial empty figure
        build_figure([], self.fig)
        self.canvas.draw()

        # Status bar
        ttk.Label(self.root, textvariable=self.status,
                  relief=tk.SUNKEN, anchor='w', padding=4).pack(
                      fill=tk.X, side=tk.BOTTOM)

    # ---------- helpers ----------

    def _browse(self):
        d = filedialog.askdirectory(
            title=f"Select parent dir with *{CSV_SUFFIX}.csv files")
        if d:
            self.parent_dir.set(d)
            self._load()

    def _load(self):
        d = self.parent_dir.get()
        if not d or not os.path.isdir(d):
            messagebox.showwarning("No directory",
                                   "Pick a parent directory first.")
            return

        csvs = find_csvs(d)
        if not csvs:
            self.status.set(f"No *{CSV_SUFFIX}.csv found in {d}")
            messagebox.showinfo(
                "Nothing found",
                f"No *{CSV_SUFFIX}.csv files in:\n{d}\n\n"
                f"Run the extractor on this directory first.")
            self.csv_paths = []
            self.day_data = {}
            self.skipped = []
            self._rebuild_checklist()
            return

        self.csv_paths = csvs
        self.day_data = {}
        self.day_vars = {}
        self.skipped = []

        ok = 0
        for csv_path in csvs:
            df = load_day_csv(csv_path)
            if df is None:
                self.skipped.append(
                    (Path(csv_path).name,
                     f"unreadable, empty, or missing column "
                     f"{VALLEY_COL}/{PEAK_COL}"))
                continue
            stats = per_day_stats(df)
            if stats is None:
                self.skipped.append(
                    (Path(csv_path).name,
                     f"fewer than {MIN_VALID_CYCLES} valid cycles"))
                continue
            date_str = extract_date(csv_path)
            self.day_data[date_str] = (csv_path, df, stats)
            ok += 1

        self._rebuild_checklist()

        msg = f"Loaded {ok} day(s) from {Path(d).name}"
        if self.skipped:
            msg += f" ({len(self.skipped)} skipped)"
        self.status.set(msg + "  —  press Plot to render.")

        # Silently dropping a recording day is an analysis decision, not a
        # cosmetic detail. Surface which days and why, not just a count.
        if self.skipped:
            detail = "\n".join(f"  {name}: {why}" for name, why in self.skipped)
            messagebox.showwarning(
                "Days skipped",
                f"{len(self.skipped)} day(s) were not loaded:\n\n{detail}\n\n"
                "Any day absent from a published figure must be accounted for "
                "in the manuscript.")

    def _rebuild_checklist(self):
        for child in self.list_frame.winfo_children():
            child.destroy()
        self.day_vars = {}
        if not self.day_data:
            ttk.Label(self.list_frame, text="(no days loaded)",
                      foreground='#888').pack(padx=4, pady=4)
            return

        for date_str in sorted(self.day_data.keys()):
            csv_path, df, stats = self.day_data[date_str]
            v = tk.BooleanVar(value=True)
            self.day_vars[date_str] = v
            label = (f"{date_str}   n={stats['n_total']}   "
                     f"v={stats['valley_median']:.1f}  "
                     f"gap={stats['gap_median']:.1f}")
            if stats['n_blowout'] > 0:
                label += f"  !{stats['n_blowout']}"
            ttk.Checkbutton(self.list_frame, text=label, variable=v,
                            command=self._plot).pack(anchor='w', padx=2)

    def _toggle_all(self, val):
        for v in self.day_vars.values():
            v.set(val)
        self._plot()

    def _selected_panels(self):
        out = []
        for date_str in sorted(self.day_data.keys()):
            v = self.day_vars.get(date_str)
            if v is None or not v.get():
                continue
            csv_path, df, stats = self.day_data[date_str]
            out.append((date_str, df, stats))
        return out

    def _plot(self):
        if not self.day_data:
            self.status.set("Nothing loaded — Browse / Load CSVs first.")
            return

        panels = self._selected_panels()
        ncols = max(1, int(self.ncols_var.get()))
        clip = self.clip_var.get()
        suffix = (Path(self.parent_dir.get()).name
                  if self.parent_dir.get() else None)

        build_figure(panels, self.fig, ncols=ncols, title_suffix=suffix,
                     clip_blowouts=clip,
                     empty_message="No days selected.")

        self.canvas.draw_idle()

        n_excluded = len(self.day_data) - len(panels)
        msg = (f"Plotted {len(panels)} day(s) "
               f"({ncols} col{'s' if ncols != 1 else ''}, "
               f"clip={'on' if clip else 'off'})")
        if n_excluded:
            msg += f"  —  {n_excluded} day(s) excluded by checklist"
        self.status.set(msg + ".")

    def _save(self):
        if not self.day_data:
            messagebox.showinfo("Nothing to save", "Plot something first.")
            return
        default = f"ribbon{CSV_SUFFIX}.png"
        if self.parent_dir.get():
            default = os.path.join(self.parent_dir.get(), default)
        path = filedialog.asksaveasfilename(
            title="Save figure", defaultextension=".png",
            initialfile=os.path.basename(default),
            initialdir=os.path.dirname(default) or None,
            filetypes=[("PNG", "*.png"), ("PDF", "*.pdf"),
                       ("SVG", "*.svg"), ("All", "*.*")])
        if not path:
            return
        try:
            # 300 dpi: MDPI requires >=300 dpi for raster figures.
            self.fig.savefig(path, dpi=300, bbox_inches='tight')
            self.status.set(f"Saved: {path}")
        except Exception as e:
            messagebox.showerror("Save failed", str(e))

    def _blowout_report(self):
        if not self.day_data:
            messagebox.showinfo("Nothing loaded", "Load CSVs first.")
            return
        win = tk.Toplevel(self.root)
        win.title("Blowout report")
        win.geometry("900x600")

        frame = ttk.Frame(win)
        frame.pack(fill=tk.BOTH, expand=True)
        txt = tk.Text(frame, font=('Courier', 9), wrap='none',
                      bg='#1a1a2e', fg='#dcdcdc')
        sb = ttk.Scrollbar(frame, orient='vertical', command=txt.yview)
        txt.config(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        txt.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        thresh = BLOWOUT_THRESH
        txt.insert('end',
                   f"BLOWOUT REPORT  (peak, valley or gap > {thresh:.0f} cm)\n"
                   + "=" * 70 + "\n\n")

        for date_str in sorted(self.day_data.keys()):
            csv_path, df, stats = self.day_data[date_str]
            peaks = df[PEAK_COL].values
            valleys = df[VALLEY_COL].values
            gaps = peaks - valleys
            mask = (peaks > thresh) | (valleys > thresh) | (gaps > thresh)
            n_bad = int(mask.sum())
            if n_bad == 0:
                txt.insert('end',
                           f"  {date_str}: clean  ({len(df)} cycles)\n")
                continue
            pct = 100 * n_bad / len(df) if len(df) else 0
            txt.insert('end',
                       f"\n  {date_str}: {len(df)} cycles, "
                       f"{n_bad} blowouts ({pct:.1f}%)\n")
            if 'file_name' in df.columns:
                bad = df[mask]
                for fname, g in bad.groupby('file_name'):
                    idxs = g.index.tolist()
                    p_max = float(g[PEAK_COL].max())
                    v_max = float(g[VALLEY_COL].max())
                    txt.insert('end',
                               f"      {fname}: {len(g):>3d} bad cycles  "
                               f"(idx {idxs[0]}–{idxs[-1]})  "
                               f"max p={p_max:.1f}cm  v={v_max:.1f}cm\n")
            else:
                txt.insert('end',
                           "      (no 'file_name' column — cannot attribute "
                           "blowouts to source recordings)\n")
        txt.config(state='disabled')


# ============================================================================
# main
# ============================================================================

def main():
    root = tk.Tk()
    app = RibbonGUI(root)
    # If a directory was passed on the command line, auto-load
    if len(sys.argv) > 1 and os.path.isdir(sys.argv[1]):
        app.parent_dir.set(sys.argv[1])
        root.after(100, app._load)
    root.mainloop()


if __name__ == "__main__":
    main()
