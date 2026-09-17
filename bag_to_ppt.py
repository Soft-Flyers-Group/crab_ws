#!/usr/bin/env python3

"""Create a graph-heavy PowerPoint containing every CRAB flipper bag/run.

Deck order:
    gait (simple to complex) -> parameters -> model runs -> flipper -> measured runs -> comparisons

Each run gets a full-screen 16:9 graph slide containing:
    1. Smoothed flattened load-cell Fx/Fy/Fz plus dotted resultant magnitude
       (torques are not plotted)
    2. Zero-repaired Servo 1 and Servo 2 commands/encoders, without waveform smoothing

Smoothing uses time-based Hampel outlier rejection followed by a rolling
median. This removes isolated spikes before smoothing the remaining variation.

By default, a first pass calculates shared force and servo limits from the
cleaned signals so every run is visually comparable. Each run retains its own
recorded time range. Use
--independent-axes only when close-up autoscaled plots are specifically needed.
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path
import re
from tempfile import TemporaryDirectory

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN
from pptx.util import Inches, Pt
from scipy.cluster.vq import kmeans2
from scipy.ndimage import median_filter
from scipy.signal import butter, coherence, csd, detrend, sosfiltfilt, welch

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from crab_interfaces.msg import LoadCell, ServoData
except ImportError:
    rosbag2_py = None
    LoadCell = ServoData = None


FLIPPERS = ("control", "fiberglass", "petg_thin", "rib_flipper", "stripe_flipper")
TOPICS = {
    "/load_cell_data": LoadCell,
    "/servo/position_data": ServoData,
    "/servo/encoder_data": ServoData,
}
LOAD_LABELS = ("Fx", "Fy", "Fz")
LOAD_COLORS = ("#d62728", "#2ca02c", "#1f77b4")
NET_FORCE_COLOR = "#111827"
SERVO_COLORS = ("#1f77b4", "#ff7f0e")
MAX_PLOT_POINTS = 15_000

SLIDE_W = Inches(13.333333)
SLIDE_H = Inches(7.5)
NAVY = RGBColor(18, 31, 53)
BLUE = RGBColor(61, 141, 255)
LIGHT_BLUE = RGBColor(208, 237, 250)
WHITE = RGBColor(255, 255, 255)
GRAY = RGBColor(92, 101, 114)


# Run catalogue transcribed from Gait Analysis.pdf, parameter tables, pages 2-11.
# Table row N maps to filename run N; this is a declared assumption, not verified provenance.
import json
import sys
import textwrap
from matplotlib.backends.backend_pdf import PdfPages

GAIT_ORDER = ('DOUBLESIN', 'experimental_sinusoidalYaw', 'SIN', 'SINmiddle',
              'yawPower', 'rollAmpAndFrequency', 'NESTEDSIN', 'SLOWFAST',
              'SINFOURIER', 'FOURIER')
GAIT_DISPLAY_NAMES = {
    'DOUBLESIN': 'Sin (U) | Sin (U)',
    'experimental_sinusoidalYaw': 'Sin (LNU) | Sin (LU)',
    'SIN': 'Sin (U) | Square (R)',
    'SINmiddle': 'Sin (HU) | Square (R)',
    'yawPower': 'Sin (LU) | Square (LNR)',
    'rollAmpAndFrequency': 'Sin (LLU) | Square (LNR)',
    'NESTEDSIN': 'Sin (NU) | Square (R)',
    'SLOWFAST': 'Sin (NU) (Slow/Fast) | Square (R)',
    'SINFOURIER': 'Sin (U) | Fourier',
    'FOURIER': 'Fourier | Square (R)',
}
GAIT_COMPACT_NAMES = {k: v.replace(' | ', ' |\n') for k, v in GAIT_DISPLAY_NAMES.items()}
GAIT_NAME_LEGEND = 'U = Uniform; N = Not; L = Lower amplitude; H = Higher amplitude; R = Regular'
PARAMETER_SOURCE = 'pdf'
RUN_PARAMETERS = {}

def _register(gait, frequencies, **base):
    RUN_PARAMETERS[gait] = [dict(gait=gait, run=i+1, frequency_hz=f,
        table_frequency_hz=f, center1=2500, center2=2000, amplitude1=500,
        amplitude2=1000, direction=1, dt=0.02, initial1=2500, initial2=2000,
        low_threshold=2200, high_threshold=2800, low_command=1000,
        high_command=2000, harmonics=299, notes='', **{}) for i, f in enumerate(frequencies)]
    for p in RUN_PARAMETERS[gait]:
        p.update(base)

_register('DOUBLESIN', [0.4, 0.67], kind='double', page=9)
RUN_PARAMETERS['DOUBLESIN'][1]['notes'] = '0.67 Hz used literally from table; exact controller omega for this row was not provided.'
_register('experimental_sinusoidalYaw', [0.5]*8, kind='piecewise_sine', page=2,
          center1=2048, center2=1000, amplitude1=75, initial1=2048, initial2=1000)
for p, a, fraction in zip(RUN_PARAMETERS['experimental_sinusoidalYaw'],
                          [22.5,30,45,45]*2, [0.7,0.7,0.7,0.5]*2):
    p.update(amplitude2=a, power_fraction=fraction)
    p['notes'] = 'Rows 5-8 repeat settings 1-4; retained as separate runs.'
_register('SIN', [0.5,0.75], kind='cosine_switch', page=6)
_register('SINmiddle', [0.5], kind='cosine_switch', page=8, amplitude1=850,
          center1=2200, initial1=2200, dt=0.015, low_threshold=1550,
          high_threshold=2850, low_command=1700, high_command=2700)
_register('yawPower', [0.5]*4, kind='piecewise_square', page=4,
          amplitude1=75, center1=2048, center2=1000, power_fraction=0.7,
          recovery_angle=5, initial1=2048, initial2=1000)
for p,a in zip(RUN_PARAMETERS['yawPower'], [45,30,60,90]): p['power_angle']=a
_register('rollAmpAndFrequency', [0.5,0.75,1,1,0.75,0.5,0.5,0.75,1,0.5,0.75,1],
          kind='piecewise_square', page=3, center1=2048, center2=1000,
          power_fraction=0.7, recovery_angle=5, power_angle=45, initial1=2048, initial2=1000)
for p,a in zip(RUN_PARAMETERS['rollAmpAndFrequency'], [50]*3+[60]*3+[70]*3+[80]*3): p['amplitude1']=a
_register('NESTEDSIN', [4/(2*np.pi),6/(2*np.pi)], kind='nested', page=5)
RUN_PARAMETERS['NESTEDSIN'][0].update(center1=2550,direction=-1,speed=4,modulation=0.6,
                                    low_threshold=2120,high_threshold=2880,table_frequency_hz=0.637)
RUN_PARAMETERS['NESTEDSIN'][1].update(center1=2500,direction=1,speed=6,modulation=0.7,
    table_frequency_hz=0.955,notes='Conflict: table center1=2500, direction=+1; pasted controller uses 2550, -1.')
_register('SLOWFAST', [0.65,0.85], kind='slowfast', page=7, harmonics=99)
RUN_PARAMETERS['SLOWFAST'][0].update(low_threshold=2120,high_threshold=2880)
for p in RUN_PARAMETERS['SLOWFAST']:
    p['notes']='Conflict: table direction=+1; pasted controllers use -1. Formula image also uses -1.'
_register('SINFOURIER', [0.4,0.65], kind='sin_fourier', page=10, amplitude2=1300)
for p in RUN_PARAMETERS['SINFOURIER']:
    p['notes']='Conflict: table center2=2000; formula image and controller use 2500. Series scale 1300 comes from controller.'
_register('FOURIER', [0.35,0.65], kind='fourier_switch', page=11, amplitude1=1250,
          low_threshold=2100,high_threshold=2900)


def parameters_for(gait, run, source=None):
    rows = RUN_PARAMETERS.get(gait, [])
    if run < 1 or run > len(rows): return None
    p = dict(rows[run-1])
    source = source or PARAMETER_SOURCE
    p['parameter_source'] = source
    if source == 'code':
        if gait == 'NESTEDSIN' and run == 2: p.update(center1=2550,direction=-1)
        if gait == 'SLOWFAST': p['direction']=-1
        if gait == 'SINFOURIER': p['center2']=2500
    return p


def gait_sort_key(value):
    family, _, row = str(value).partition('::')
    return (GAIT_ORDER.index(family) if family in GAIT_ORDER else 999,
            family, int(row) if row.isdigit() else 0)


def harmonic_sum(t, amplitude, omega, nmax, slowfast=False):
    result = np.zeros_like(np.asarray(t, dtype=float))
    for n in range(1,nmax+1):
        if slowfast: result += amplitude*np.sin(n*omega*t)/n
        else: result += (-1)**(n+1)*4/np.pi*amplitude*np.cos((2*n-1)*omega*t)/(2*n+1)
    return result


def calculate_commands(p, t):
    """Mathematical target functions; switch channel uses ideal S1 feedback only."""
    t=np.asarray(t,dtype=float); w=2*np.pi*p['frequency_hz']; a=p['amplitude1']
    kind=p['kind']; c1=p['center1']; c2=p['center2']; d=p['direction']
    if kind.startswith('piecewise'):
        u=(t*p['frequency_hz']) % 1; b=p['power_fraction']; power=u<b
        s=np.where(power,u/b,(u-b)/(1-b))
        angle1=np.where(power,-a*np.cos(np.pi*s),a*np.cos(np.pi*s))
        if kind=='piecewise_sine':
            angle2=np.where(power,-p['amplitude2']*np.sin(np.pi*s),p['amplitude2']*np.sin(np.pi*s))
        else: angle2=np.where(power,p['power_angle'],p['recovery_angle'])
        return np.clip(np.trunc(c1+4096/360*angle1),0,4095), np.clip(np.trunc(c2+4096/360*angle2),0,4095)
    if kind=='nested': x=c1+np.rint(d*a*np.sin(p['speed']*t+p['modulation']*np.sin(p['speed']*t)))
    elif kind in ('slowfast','fourier_switch'):
        x=c1+d*5*np.rint(harmonic_sum(t,a,w,p['harmonics'],kind=='slowfast')/5)
    else: x=np.rint(c1+d*a*np.cos(w*t))
    if kind=='double': y=np.rint(c2+d*p['amplitude2']*np.cos(w*t))
    elif kind=='sin_fourier': y=c2+5*np.rint(harmonic_sum(t,p['amplitude2'],w,p['harmonics'])/5)
    else:
        state=p['initial2']; y=np.empty_like(x)
        for i, e in enumerate(x):
            if e>p['high_threshold']: state=p['low_command']
            if e<p['low_threshold']: state=p['high_command']
            y[i]=state
    return x,y


def sampled_commands(p, cycles=2):
    """Nominal timer simulation. Stored-command controllers publish before update.
    Ideal encoder assumption: latest feedback equals most recently sent S1.
    Does not represent motor dynamics, communication delays or measured motion.
    """
    t=np.arange(p['dt'], cycles/p['frequency_hz']+p['dt']/2,p['dt'])
    x,y=calculate_commands(p,t)
    if p['kind'].startswith('piecewise'): return t,x,y
    x_sent=np.r_[p['initial1'],x[:-1]]
    if p['kind'] in ('double','sin_fourier'):
        return t,x_sent,np.r_[p['initial2'],y[:-1]]
    state=p['initial2']; sent=[]
    for e in x_sent:
        sent.append(state)
        if e>p['high_threshold']: state=p['low_command']
        if e<p['low_threshold']: state=p['high_command']
    return t,x_sent,np.array(sent)


def equation_lines(p):
    c=p['center1']; a=p['amplitude1']; d=p['direction']; w=f"{2*p['frequency_hz']:.6g}πt"
    kind=p['kind']; c2=p['center2']
    if kind.startswith('piecewise'):
        b=p['power_fraction']; f=p['frequency_hz']
        first=f"u=(t×{f:g}) mod 1; b={b:g}; s=u/b during power, (u-b)/(1-b) during recovery."
        second=f"θ1=-{a:g} cos(πs) during power; +{a:g} cos(πs) during recovery."
        third=(f"θ2=-{p['amplitude2']:g} sin(πs) during power; +{p['amplitude2']:g} sin(πs) during recovery."
               if kind=='piecewise_sine' else f"θ2={p['power_angle']:g}° during power; {p['recovery_angle']:g}° during recovery.")
        return [first, second, third, f"p1=clip(int({c}+4096θ1/360)); p2=clip(int({c2}+4096θ2/360)); clip to [0,4095]."]
    if kind=='nested': first=f"p1={c}+round({d*a:g} sin({p['speed']}t+{p['modulation']} sin({p['speed']}t)))"
    elif kind=='slowfast': first=f"p1={c}+({d})×5 round([Σ(n=1..99) {a:g} sin(n×{w})/n]/5)"
    elif kind=='fourier_switch': first=f"p1={c}+5 round(F(t)/5); F=(4×{a:g}/π) Σ(n=1..299) (-1)^(n+1) cos((2n-1)×{w})/(2n+1)"
    else: first=f"p1=round({c}+{d*a:g} cos({w}))"
    if kind=='double': second=f"p2=round({c2}+{d*p['amplitude2']:g} cos({w}))"
    elif kind=='sin_fourier': second=f"p2={c2}+5 round(F(t)/5); F=(4×{p['amplitude2']:g}/π) Σ(n=1..299) (-1)^(n+1) cos((2n-1)×{w})/(2n+1)"
    else: second=f"q2[k+1]={p['low_command']} if e1[k]>{p['high_threshold']}; {p['high_command']} if e1[k]<{p['low_threshold']}; otherwise q2[k]."
    return [first,second]


def counts_to_degrees(values):
    """Encoder-zero reference: counts * 360/4096. No physical neutral assumed."""
    return np.asarray(values,dtype=float) * 360.0 / 4096.0


def parameter_summary(p):
    if not p: return 'No parameter table row matches this run.'
    a1=p['amplitude1'] if p['kind'].startswith('piecewise') else float(counts_to_degrees(p['amplitude1']))
    label='scale' if p['kind'] in ('slowfast','fourier_switch') else 'amplitude'
    text=f"f={p['frequency_hz']:.6g} Hz; S1 {label}={a1:.2f}°; centers={float(counts_to_degrees(p['center1'])):.2f}° / {float(counts_to_degrees(p['center2'])):.2f}°"
    if 'power_fraction' in p: text+=f"; power={p['power_fraction']:g}"
    else:text+=f"; direction={p['direction']}"
    if 'modulation' in p: text+=f"; speed={p['speed']}; modulation={p['modulation']}"
    return text


def make_parameter_figure(p):
    fig,axs=plt.subplots(2,1,figsize=(16,9),sharex=True)
    fig.suptitle(f"{GAIT_DISPLAY_NAMES[p['gait']]}   /   Run {p['run']}",x=.065,y=.97,ha='left',fontsize=23,fontweight='bold')
    fig.text(.065,.916,parameter_summary(p),fontsize=11)
    fig.text(.065,.882,'MODEL ONLY   •   Table settings + controller equations   •   No measured force or motion',fontsize=11,color='#8a4b08')
    t=np.linspace(0,2/p['frequency_hz'],12000)
    x,y=calculate_commands(p,t); ts,xs,ys=sampled_commands(p)
    alternate=None
    if 'Conflict:' in p['notes']:
        other=parameters_for(p['gait'],p['run'],'code' if p['parameter_source']=='pdf' else 'pdf')
        alternate=calculate_commands(other,t)
    for i,(v,vs) in enumerate(((x,xs),(y,ys))):
        axs[i].plot(t,counts_to_degrees(v),color=SERVO_COLORS[i],lw=1.25,label='Calculated target / ideal-feedback switch')
        axs[i].step(ts,counts_to_degrees(vs),where='post',lw=.85,color='#111827',alpha=.65,label='Nominal timer commands, ideal feedback')
        if alternate is not None: axs[i].plot(t,counts_to_degrees(alternate[i]),'--',color='#ab3348',lw=.9,label='Alternate source (controller vs table)')
        axs[i].set_ylabel(f'Servo {i+1}\nEncoder angle (deg)',fontsize=12)
        axs[i].set_ylim(0,370); axs[i].grid(alpha=.2)
        axs[i].legend(loc='upper right',fontsize=8,ncol=3)
    axs[1].set_xlabel('Time since controller startup (s)',fontsize=12)
    lines=equation_lines(p)
    footer='\n'.join(lines) + '\nPlotted encoder angle = p × 360/4096 degrees. Above position equations retain raw counts for exact rounding.'
    if p['notes']: footer+='\n'+p['notes']
    fig.text(.065,.19,footer,fontsize=9,va='top',linespacing=1.5)
    fig.text(.065,.025,f"Source: Gait Analysis.pdf p.{p['page']}, row {p['run']}; active policy: {p['parameter_source']}. Servo IDs authoritative; PDF labels S1 Roll / S2 Yaw.",fontsize=8,color='#555555')
    fig.subplots_adjust(left=.085,right=.97,top=.835,bottom=.27,hspace=.20)
    return fig


def export_parameter_report(directory):
    directory=Path(directory); directory.mkdir(parents=True,exist_ok=True)
    figures=directory/'parameter_plots'; figures.mkdir(exist_ok=True)
    allrows=[parameters_for(g,r) for g in GAIT_ORDER for r in range(1,len(RUN_PARAMETERS[g])+1)]
    for p in allrows:
        p['center1_deg']=float(counts_to_degrees(p['center1']))
        p['center2_deg']=float(counts_to_degrees(p['center2']))
        p['amplitude1_deg_or_scale']=p['amplitude1'] if p['kind'].startswith('piecewise') else float(counts_to_degrees(p['amplitude1']))
        p['amplitude2_deg_or_scale']=p['amplitude2'] if p['kind'].startswith('piecewise') else float(counts_to_degrees(p['amplitude2']))
        p['angle_reference']='encoder zero; degrees = counts * 360/4096'
    with (directory/'run_parameters.json').open('w') as f: json.dump(allrows,f,indent=2)
    cols=sorted({k for p in allrows for k in p})
    with (directory/'run_parameters.csv').open('w',newline='') as f:
        wr=csv.DictWriter(f,fieldnames=cols);wr.writeheader();wr.writerows(allrows)
    out=directory/'gait_parameter_analysis.pdf'
    with PdfPages(out) as pdf:
        fig=plt.figure(figsize=(16,9))
        fig.text(.07,.88,'Gait analysis by parameter row',fontsize=30,fontweight='bold')
        notes=[
            '37 runs in 10 gait families. Each table row is treated as the corresponding filename run number.',
            'Order: Sin-Sin, piecewise Sin-Sin, ordinary Sin-Square, wide Sin-Square,',
            'fixed-amplitude and varying-amplitude timed Sin-Square, Nested Sin, Slow/Fast, Sin-Fourier, Fourier-Square.',
            'These plots reconstruct commanded motion. ROS bags / force CSVs were not supplied.',
            'Force comparisons, tracking errors and measured spectra must be regenerated from recorded data.',
            'Defaults follow PDF TABLES. Dashed alternatives show conflicts with pasted controller code.',
            'Nested Sin row 2: table center 2500 / direction +1 vs controller 2550 / -1.',
            'Slow/Fast rows: table direction +1 vs controller -1. Sin-Fourier: table center2 2000 vs controller 2500.',
            'Double Sin row 2 uses 0.67 Hz literally. Nested Sin frequencies use exact speed/(2π).',
            'Missing table fields come from the corresponding pasted controller / PDF equations.',
            'Square switching uses ideal S1 feedback in these models. Actual delays and tracking change switching times.',
            'Fourier retains denominator 2n+1 and 299 terms. Slow/Fast uses 99 terms; series scale is not peak amplitude.',
            'High harmonics exceed the timer Nyquist frequency. Dense targets and sampled commands can differ.',
            'All servo axes and tables use degrees = counts × 360/4096, from encoder zero. Physical neutral is unverified.',
            GAIT_NAME_LEGEND,
        ]
        for i,line in enumerate(notes):fig.text(.07,.79-i*.043,line,fontsize=12)
        pdf.savefig(fig);plt.close(fig)
        for gait in GAIT_ORDER:
            rows=[p for p in allrows if p['gait']==gait]
            fig=plt.figure(figsize=(16,9)); ax=fig.add_axes([.065,.20,.87,.60]);ax.axis('off')
            fig.text(.065,.91,GAIT_DISPLAY_NAMES[gait],fontsize=26,fontweight='bold')
            fig.text(.065,.855,f"Original key: {gait}  /  PDF p.{rows[0]['page']}  /  rows map to run numbers",fontsize=12)
            headers=['Run','f (Hz)','A1 (deg)','C1 (deg)','A2 / power (deg)','C2 (deg)','Direction','Power ratio','Extra']
            data=[]
            for p in rows:
                extra=(f"speed {p['speed']}, omega {p['modulation']}" if 'speed' in p else
                       f"limits {float(counts_to_degrees(p['low_threshold'])):.1f}/{float(counts_to_degrees(p['high_threshold'])):.1f}" if p['kind'] in ('cosine_switch','slowfast','fourier_switch') else f"recovery {p['recovery_angle']} deg" if 'recovery_angle' in p else '')
                a1=p['amplitude1'] if p['kind'].startswith('piecewise') else float(counts_to_degrees(p['amplitude1']))
                if p['kind']=='piecewise_square': a2=f"{p['power_angle']:.2f}"
                elif p['kind']=='piecewise_sine': a2=f"{p['amplitude2']:.2f}"
                elif p['kind'] in ('double','sin_fourier'): a2=f"{float(counts_to_degrees(p['amplitude2'])):.2f}"
                else: a2=f"{float(counts_to_degrees(p['low_command'])):.1f}/{float(counts_to_degrees(p['high_command'])):.1f}"
                data.append([p['run'],f"{p['frequency_hz']:.6g}",f"{a1:.2f}",f"{float(counts_to_degrees(p['center1'])):.2f}",a2,f"{float(counts_to_degrees(p['center2'])):.2f}",('—' if p['kind'].startswith('piecewise') else p['direction']),p.get('power_fraction','—'),extra])
            table=ax.table(cellText=data,colLabels=headers,loc='center',cellLoc='center',colWidths=[.05,.09,.08,.08,.13,.08,.09,.10,.20])
            table.auto_set_font_size(False);table.set_fontsize(11);table.scale(1,2)
            for (r,c),cell in table.get_celld().items():
                cell.set_edgecolor('#dddddd')
                if r==0:cell.set_facecolor('#182b49');cell.set_text_props(color='white',weight='bold')
            fig.text(.065,.125,'All angles above are degrees. Centers use encoder zero; amplitudes are offsets. Harmonic amplitudes are scale factors.\nDegrees = counts × 360/4096. Raw count parameters are retained in the CSV/JSON and exact position equations.',fontsize=11)
            png=figures/f'{GAIT_ORDER.index(gait)+1:02d}_{gait}_table.png';fig.savefig(png,dpi=120);pdf.savefig(fig);plt.close(fig)
            for p in rows:
                fig=make_parameter_figure(p)
                fig.savefig(figures/f"{GAIT_ORDER.index(gait)+1:02d}_{gait}_run_{p['run']:02d}.png",dpi=120)
                pdf.savefig(fig);plt.close(fig)
    print(f'Wrote {out} with {len(allrows)} model runs. No measured force analysis performed.')
    return out


def natural_key(value):
    return [
        int(part) if part.isdigit() else part.lower()
        for part in re.split(r"(\d+)", str(value))
    ]


def identify_flipper(filename):
    lower = filename.lower()
    for flipper in FLIPPERS:
        if lower.startswith(flipper.lower()):
            return flipper
    return None


def parse_gait_and_run(path, flipper):
    stem = path.stem
    if flipper == "fiberglass":
        experiment = stem[len("fiberglass"):].lstrip("_")
        match = re.match(r"^(.*?)(\d+)$", experiment)
        return (
            (match.group(1), int(match.group(2)))
            if match
            else (experiment or "unknown", 1)
        )

    prefix = f"{flipper}_"
    suffix = f"_{flipper}"
    experiment = (
        stem[len(prefix):]
        if stem.lower().startswith(prefix.lower())
        else stem
    )
    if experiment.lower().endswith(suffix.lower()):
        experiment = experiment[:-len(suffix)]
    if experiment.lower().startswith("vansh_"):
        experiment = experiment[len("vansh_"):]
    match = re.match(r"^(.*)_(\d+)$", experiment)
    return (
        (match.group(1), int(match.group(2)))
        if match
        else (experiment or "unknown", 1)
    )


def friendly(value):
    family, sep, row = str(value).partition('::')
    names = {'control':'Control', 'fiberglass':'Fiberglass', 'petg_thin':'PETG Thin',
             'rib_flipper':'Rib Flipper', 'stripe_flipper':'Stripe Flipper'}
    label = GAIT_DISPLAY_NAMES.get(family, names.get(family, family))
    return label + (f" / Run {row}" if sep else '')


def compact_gait_name(value):
    family, sep, row = str(value).partition('::')
    if sep:
        p = parameters_for(family, int(row))
        return f"Run {row}" + (f"\n{p['frequency_hz']:.3g} Hz" if p else '')
    return GAIT_COMPACT_NAMES.get(family, friendly(value))


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def flatten_load_batches(batch_times, batches):
    if not batches:
        return np.empty(0), np.empty((0, 6), dtype=np.float32)

    times = np.asarray(batch_times, dtype=np.float64)
    positive_periods = np.diff(times)
    positive_periods = positive_periods[positive_periods > 0]
    fallback = float(np.median(positive_periods)) if positive_periods.size else 0.0
    expanded_times = []
    expanded_values = []

    for index, batch in enumerate(batches):
        rows = batch.shape[0]
        period = (
            times[index + 1] - times[index]
            if index + 1 < len(times)
            else fallback
        )
        if period > 0:
            row_times = times[index] + np.arange(rows) * (period / rows)
        else:
            row_times = np.full(rows, times[index])
        expanded_times.append(row_times)
        expanded_values.append(batch)

    return (
        np.concatenate(expanded_times),
        np.concatenate(expanded_values, axis=0),
    )


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    values = {
        "load_t": [],
        "load": [],
        "cmd_t": [],
        "cmd": [],
        "enc_t": [],
        "enc": [],
    }

    while reader.has_next():
        topic, raw, _ = reader.read_next()
        message_type = TOPICS.get(topic)
        if message_type is None:
            continue

        msg = deserialize_message(raw, message_type)
        time = stamp_to_sec(msg.header.stamp)

        if topic == "/load_cell_data":
            matrix = np.asarray(msg.data, dtype=np.float32)
            expected = msg.rows * msg.cols
            if matrix.size != expected:
                raise ValueError(
                    f"LoadCell has {matrix.size} values; expected {expected}"
                )
            values["load_t"].append(time)
            values["load"].append(matrix.reshape(msg.rows, msg.cols))
        elif topic == "/servo/position_data":
            values["cmd_t"].append(time)
            values["cmd"].append(np.asarray(msg.data, dtype=np.float32))
        elif topic == "/servo/encoder_data":
            values["enc_t"].append(time)
            values["enc"].append(np.asarray(msg.data, dtype=np.float32))

    load_t, load = flatten_load_batches(values["load_t"], values["load"])
    all_times = load_t.tolist() + values["cmd_t"] + values["enc_t"]
    time_zero = min(all_times) if all_times else 0.0

    return {
        "load_t": load_t - time_zero,
        "load": load,
        "cmd_t": np.asarray(values["cmd_t"], dtype=np.float64) - time_zero,
        "cmd": np.asarray(values["cmd"], dtype=np.float32),
        "enc_t": np.asarray(values["enc_t"], dtype=np.float64) - time_zero,
        "enc": np.asarray(values["enc"], dtype=np.float32),
    }


def downsample(time, values, maximum=MAX_PLOT_POINTS):
    if len(time) <= maximum:
        return time, values
    step = int(np.ceil(len(time) / maximum))
    return time[::step], values[::step]


def odd_window(seconds, sample_rate, minimum=3):
    """Convert a time duration to an odd number of samples."""
    samples = max(minimum, int(round(seconds * sample_rate)))
    return samples if samples % 2 else samples + 1


def smooth_traces(times, values, hampel_seconds=0.10,
                  smooth_seconds=0.25, sigma=3.5):
    """Hampel-clean and rolling-median smooth one or more signal columns."""
    times = np.asarray(times, dtype=np.float64)
    values = np.asarray(values, dtype=np.float64)

    was_1d = values.ndim == 1
    if was_1d:
        values = values[:, np.newaxis]

    if values.ndim != 2:
        raise ValueError(f"Expected 1-D or 2-D trace data, got shape {values.shape}")
    if len(times) != values.shape[0]:
        raise ValueError(
            f"Timestamp count {len(times)} does not match sample count "
            f"{values.shape[0]}"
        )
    if len(times) < 3:
        return values[:, 0] if was_1d else values.copy()

    positive_dt = np.diff(times)
    positive_dt = positive_dt[positive_dt > 0]
    if not positive_dt.size:
        return values[:, 0] if was_1d else values.copy()

    sample_rate = 1.0 / float(np.median(positive_dt))
    hampel_n = odd_window(hampel_seconds, sample_rate)
    smooth_n = odd_window(smooth_seconds, sample_rate)
    result = values.copy()

    for column in range(values.shape[1]):
        signal = values[:, column].copy()
        valid = np.isfinite(signal)
        if np.count_nonzero(valid) < 3:
            continue

        if not np.all(valid):
            valid_indices = np.flatnonzero(valid)
            missing_indices = np.flatnonzero(~valid)
            signal[~valid] = np.interp(
                missing_indices,
                valid_indices,
                signal[valid],
            )

        local_median = median_filter(signal, size=hampel_n, mode="nearest")
        deviation = np.abs(signal - local_median)
        local_mad = median_filter(deviation, size=hampel_n, mode="nearest")
        robust_sigma = 1.4826 * local_mad

        global_mad = np.median(np.abs(signal - np.median(signal)))
        sigma_floor = max(1e-9, 0.01 * 1.4826 * global_mad)
        threshold = sigma * np.maximum(robust_sigma, sigma_floor)

        outliers = deviation > threshold
        signal[outliers] = local_median[outliers]
        result[:, column] = median_filter(
            signal,
            size=smooth_n,
            mode="nearest",
        )

    return result[:, 0] if was_1d else result


def repair_servo_zeros(values):
    """Replace invalid zero servo samples using neighboring valid samples.

    Interior runs of zeros are linearly interpolated between the closest
    nonzero samples on each side. Leading and trailing zeros are filled with
    the nearest available nonzero value. Each servo column is repaired
    independently.
    """
    values = np.asarray(values, dtype=np.float64)
    was_1d = values.ndim == 1
    if was_1d:
        values = values[:, np.newaxis]

    if values.ndim != 2:
        raise ValueError(f"Expected 1-D or 2-D servo data, got shape {values.shape}")

    repaired = values.copy()
    sample_indices = np.arange(repaired.shape[0])

    for column in range(repaired.shape[1]):
        signal = repaired[:, column]
        valid = np.isfinite(signal) & (signal != 0.0)

        if not np.any(valid):
            # There is no trustworthy value from which to reconstruct this
            # channel, so leave the all-zero channel unchanged.
            continue

        invalid = ~valid
        if np.any(invalid):
            signal[invalid] = np.interp(
                sample_indices[invalid],
                sample_indices[valid],
                signal[valid],
            )

    return repaired[:, 0] if was_1d else repaired


def prepare_plot_data(data, hampel_seconds=0.10, smooth_seconds=0.25,
                      hampel_sigma=3.5):
    """Clean the plotted channels once and return plot-ready arrays."""
    prepared = {
        "load_t": data["load_t"],
        "forces": np.empty((0, 3)),
        "net_force": np.empty(0),
        "cmd_t": data["cmd_t"],
        "cmd": np.empty((0, 2)),
        "enc_t": data["enc_t"],
        "enc": np.empty((0, 2)),
    }

    load = data["load"]
    if load.size and load.ndim == 2 and load.shape[1] >= 3:
        prepared["forces"] = smooth_traces(
            data["load_t"], load[:, :3], hampel_seconds,
            smooth_seconds, hampel_sigma,
        )
        # Calculate the resultant only after every component has passed through
        # the same Hampel rejection and smoothing used in the run plots.
        prepared["net_force"] = np.linalg.norm(
            prepared["forces"], axis=1
        )

    for key, time_key in (("cmd", "cmd_t"), ("enc", "enc_t")):
        servo = data[key]
        if servo.size and servo.ndim == 2:
            prepared[key] = repair_servo_zeros(servo[:, :2])
            # Preserve the actual step transitions and tracking error. No servo smoothing.
    return prepared


def padded_limits(minimum, maximum, fraction=0.05):
    """Add consistent headroom while retaining a meaningful nonzero span."""
    if not np.isfinite(minimum) or not np.isfinite(maximum):
        return None
    span = maximum - minimum
    if span <= 0:
        span = max(abs(minimum), 1.0)
    padding = span * fraction
    return minimum - padding, maximum + padding


def symmetric_limits(minimum, maximum, fraction=0.05):
    """Return limits centered on zero for fair signed-force comparisons."""
    if not np.isfinite(minimum) or not np.isfinite(maximum):
        return None
    magnitude = max(abs(minimum), abs(maximum), 1.0) * (1.0 + fraction)
    return -magnitude, magnitude


def force_diagnostic_rows(bag_path, flipper, gait, run, data, prepared):
    """Summarize raw versus final plotted forces for scale diagnosis."""
    rows = []
    raw = data["load"][:, :3] if data["load"].ndim == 2 else np.empty((0, 3))
    cleaned = prepared["forces"]
    if not cleaned.size:
        return rows

    active_start, active_stop, active_source = active_gait_interval(data)

    diagnostic_signals = [
        (axis_name, raw[:, axis_index], cleaned[:, axis_index])
        for axis_index, axis_name in enumerate(LOAD_LABELS)
    ]
    diagnostic_signals.append((
        "Net",
        np.linalg.norm(raw, axis=1),
        prepared["net_force"],
    ))

    for axis_name, raw_axis, clean_axis in diagnostic_signals:
        raw_axis = raw_axis[np.isfinite(raw_axis)]
        finite_mask = np.isfinite(clean_axis)
        clean_axis = clean_axis[finite_mask]
        clean_times = prepared["load_t"][finite_mask]
        if not clean_axis.size:
            continue

        absolute = np.abs(clean_axis)
        peak_index = int(np.argmax(absolute))
        p995_abs = float(np.percentile(absolute, 99.5))
        abs_max = float(absolute[peak_index])
        active_mask = (clean_times >= active_start) & (clean_times <= active_stop)
        if np.count_nonzero(active_mask) >= 2:
            average_times = clean_times[active_mask]
            average_absolute = absolute[active_mask]
            average_source = active_source
        else:
            average_times = clean_times
            average_absolute = absolute
            average_source = "full_load_fallback"
        duration = (
            float(average_times[-1] - average_times[0])
            if len(average_times) > 1 else 0.0
        )
        absolute_impulse = (
            float(np.trapezoid(average_absolute, x=average_times))
            if len(average_times) > 1 else 0.0
        )
        average_absolute_force = (
            absolute_impulse / duration
            if duration > 0 else float(np.mean(average_absolute))
        )
        rows.append({
            "bag": bag_path.name,
            "flipper": flipper,
            "gait": gait,
            "run": run,
            "axis": axis_name,
            "samples": len(clean_axis),
            "raw_min": float(np.min(raw_axis)) if raw_axis.size else np.nan,
            "raw_max": float(np.max(raw_axis)) if raw_axis.size else np.nan,
            "clean_min": float(np.min(clean_axis)),
            "clean_max": float(np.max(clean_axis)),
            "clean_p0_5": float(np.percentile(clean_axis, 0.5)),
            "clean_p99_5": float(np.percentile(clean_axis, 99.5)),
            "average_interval_source": average_source,
            "average_duration_s": duration,
            "clean_abs_mean": average_absolute_force,
            "clean_abs_impulse_ns": absolute_impulse,
            "clean_abs_p99_5": p995_abs,
            "clean_abs_max": abs_max,
            "max_to_p99_5_ratio": abs_max / max(p995_abs, 1e-12),
            "abs_peak_time_s": float(clean_times[peak_index]),
        })
    return rows


def write_force_diagnostics(path, rows):
    """Write one diagnostic record per bag and force axis."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0]) if rows else [
        "bag", "flipper", "gait", "run", "axis", "samples",
        "raw_min", "raw_max", "clean_min", "clean_max",
        "clean_p0_5", "clean_p99_5", "average_interval_source",
        "average_duration_s", "clean_abs_mean",
        "clean_abs_impulse_ns", "clean_abs_p99_5",
        "clean_abs_max", "max_to_p99_5_ratio", "abs_peak_time_s",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def print_suspicious_force_peaks(rows, count=30):
    """Print cleaned maxima most disproportionate to their normal range."""
    ranked = sorted(
        rows,
        key=lambda row: (row["max_to_p99_5_ratio"], row["clean_abs_max"]),
        reverse=True,
    )[:count]
    print("\nMost suspicious POST-FILTER force peaks:")
    print("ratio | abs max | p99.5 abs | time (s) | axis | bag")
    for row in ranked:
        print(
            f"{row['max_to_p99_5_ratio']:5.2f} | "
            f"{row['clean_abs_max']:7.3f} | "
            f"{row['clean_abs_p99_5']:9.3f} | "
            f"{row['abs_peak_time_s']:8.3f} | "
            f"{row['axis']:>2} | {row['bag']}"
        )


def dynamixel_to_radians(values):
    """Convert raw 0–4095 Dynamixel positions to approximately -pi–pi."""
    return (np.asarray(values, dtype=np.float64) / 4096.0) * (2.0 * np.pi) - np.pi


def hampel_clean_uniform(values, sample_rate, window_seconds=0.10, sigma=3.5):
    """Reject isolated spectral outliers without applying median smoothing."""
    values = np.asarray(values, dtype=np.float64)
    was_1d = values.ndim == 1
    if was_1d:
        values = values[:, np.newaxis]
    result = values.copy()
    window = odd_window(window_seconds, sample_rate)

    for column in range(values.shape[1]):
        signal = values[:, column].copy()
        valid = np.isfinite(signal)
        if np.count_nonzero(valid) < 3:
            continue
        if not np.all(valid):
            good = np.flatnonzero(valid)
            bad = np.flatnonzero(~valid)
            signal[bad] = np.interp(bad, good, signal[good])
        local_median = median_filter(signal, size=window, mode="nearest")
        deviation = np.abs(signal - local_median)
        local_mad = median_filter(deviation, size=window, mode="nearest")
        robust_sigma = 1.4826 * local_mad
        global_mad = np.median(np.abs(signal - np.median(signal)))
        floor = max(1e-9, 0.01 * 1.4826 * global_mad)
        outliers = deviation > sigma * np.maximum(robust_sigma, floor)
        signal[outliers] = local_median[outliers]
        result[:, column] = signal
    return result[:, 0] if was_1d else result


def active_gait_interval(data):
    """Estimate the active interval from raw Servo 1 and Servo 2."""
    candidates = []
    for key, time_key in (("cmd", "cmd_t"), ("enc", "enc_t")):
        values = data[key]
        times = data[time_key]
        if values.size and values.ndim == 2 and len(times) == len(values):
            candidates.append((times, repair_servo_zeros(values[:, :2])))
    if not candidates:
        load_t = data["load_t"]
        return (float(load_t[0]), float(load_t[-1]), "full_load") if len(load_t) else (0.0, 0.0, "empty")

    # Commands are preferred because they define the requested gait. Fall back
    # to encoders only if command data are absent.
    times, values = candidates[0]
    baseline_count = max(3, min(len(values), int(round(len(values) * 0.05))))
    baseline = np.median(values[:baseline_count], axis=0)
    amplitude = np.percentile(values, 99, axis=0) - np.percentile(values, 1, axis=0)
    threshold = np.maximum(10.0, 0.05 * amplitude)
    active = np.any(np.abs(values - baseline) > threshold, axis=1)
    if not np.any(active):
        return float(times[0]), float(times[-1]), "full_servo"

    indices = np.flatnonzero(active)
    start = float(times[indices[0]])
    stop = float(times[indices[-1]])
    # Remove transition edges when enough steady motion remains.
    if stop - start > 1.0:
        start += 0.20
        stop -= 0.20
    return start, stop, "servo_activity"


def power_spectrum(signal, sample_rate, maximum_frequency):
    """Welch PSD plus compact spectral features."""
    signal = np.asarray(signal, dtype=np.float64)
    signal = signal[np.isfinite(signal)]
    if len(signal) < 512:
        return None
    conditioned = detrend(signal, type="linear")
    target = max(256, len(conditioned) // 3)
    nperseg = min(65536, 2 ** int(np.floor(np.log2(target))))
    nperseg = min(nperseg, len(conditioned))
    nfft = max(
        nperseg,
        2 ** int(np.ceil(np.log2(sample_rate / 0.10))),
    )
    frequencies, psd = welch(
        conditioned,
        fs=sample_rate,
        window="hann",
        nperseg=nperseg,
        nfft=nfft,
        noverlap=nperseg // 2,
        detrend=False,
        scaling="density",
        average="median",
    )
    band = (frequencies >= 0.10) & (frequencies <= maximum_frequency)
    if np.count_nonzero(band) < 2:
        return None
    f = frequencies[band]
    p = psd[band]
    peak_index = int(np.argmax(p))
    total = float(np.trapezoid(p, f))
    probabilities = p / max(float(np.sum(p)), 1e-30)
    entropy = float(
        -np.sum(probabilities * np.log(probabilities + 1e-30))
        / np.log(len(probabilities))
    )
    return {
        "frequency": f,
        "psd": p,
        "dominant_hz": float(f[peak_index]),
        "peak_psd": float(p[peak_index]),
        "band_power": total,
        "spectral_entropy": entropy,
        "frequency_resolution_hz": float(sample_rate / nperseg),
        "frequency_bin_spacing_hz": float(frequencies[1] - frequencies[0]),
        "nperseg": int(nperseg),
    }


def value_near_frequency(frequencies, values, target):
    index = int(np.argmin(np.abs(frequencies - target)))
    return float(values[index])


def timestamp_quality(times):
    """Return median rate and relative timestamp jitter for a ROS stream."""
    times = np.asarray(times, dtype=np.float64)
    differences = np.diff(times)
    differences = differences[np.isfinite(differences) & (differences > 0)]
    if not differences.size:
        return np.nan, np.nan
    median_dt = float(np.median(differences))
    rate = 1.0 / median_dt
    jitter_cv = float(np.std(differences) / max(np.mean(differences), 1e-12))
    return rate, jitter_cv


def analyze_spectrum(data, bag_path, flipper, gait, run,
                     load_sample_rate=10_000.0, maximum_frequency=50.0,
                     hampel_seconds=0.10, hampel_sigma=3.5):
    """Extract time-, frequency-, and command-response features for one run."""
    result = {
        "bag": bag_path.name,
        "flipper": flipper,
        "gait": gait,
        "run": run,
        "load_sample_rate_hz": load_sample_rate,
    }
    command_rate, command_jitter = timestamp_quality(data["cmd_t"])
    encoder_rate, encoder_jitter = timestamp_quality(data["enc_t"])
    result.update({
        "command_median_rate_hz": command_rate,
        "command_timestamp_jitter_cv": command_jitter,
        "encoder_median_rate_hz": encoder_rate,
        "encoder_timestamp_jitter_cv": encoder_jitter,
    })
    load = data["load"]
    if not (load.size and load.ndim == 2 and load.shape[1] >= 3):
        result["status"] = "no_load_data"
        return result

    start, stop, interval_source = active_gait_interval(data)
    sample_times = float(data["load_t"][0]) + np.arange(len(load)) / load_sample_rate
    active = (sample_times >= start) & (sample_times <= stop)
    if np.count_nonzero(active) < 512:
        active = np.ones(len(load), dtype=bool)
        start, stop, interval_source = float(sample_times[0]), float(sample_times[-1]), "full_load_fallback"

    force_signals = {}
    force_spectra = {}
    for axis_index, axis_name in enumerate(LOAD_LABELS):
        prefix = axis_name.lower()
        raw_axis = np.asarray(load[:, axis_index], dtype=np.float64)[active]
        cleaned_axis = hampel_clean_uniform(
            raw_axis, load_sample_rate, hampel_seconds, hampel_sigma
        )
        spectrum = power_spectrum(
            cleaned_axis, load_sample_rate, maximum_frequency
        )
        if spectrum is not None:
            force_signals[prefix] = cleaned_axis
            force_spectra[prefix] = spectrum

    component_prefixes = [axis_name.lower() for axis_name in LOAD_LABELS]
    if not all(prefix in force_spectra for prefix in component_prefixes):
        result["status"] = "insufficient_active_load_data"
        return result

    # The resultant magnitude is the resultant of the already Hampel-cleaned component
    # traces. Its DC offset is removed inside power_spectrum before Welch PSD.
    net_signal = np.linalg.norm(
        np.column_stack([force_signals[prefix] for prefix in component_prefixes]),
        axis=1,
    )
    net_spectrum = power_spectrum(
        net_signal, load_sample_rate, maximum_frequency
    )
    if net_spectrum is None:
        result["status"] = "insufficient_net_force_data"
        return result
    force_signals["net"] = net_signal
    force_spectra["net"] = net_spectrum

    result.update({
        "status": "ok",
        "active_interval_source": interval_source,
        "active_start_s": start,
        "active_stop_s": stop,
        "active_duration_s": stop - start,
        "active_load_samples": int(np.count_nonzero(active)),
    })
    for prefix, signal in force_signals.items():
        spectrum = force_spectra[prefix]
        result.update({
            f"{prefix}_mean_n": float(np.mean(signal)),
            f"{prefix}_mean_abs_n": float(np.mean(np.abs(signal))),
            f"{prefix}_rms_n": float(np.sqrt(np.mean(signal ** 2))),
            f"{prefix}_std_n": float(np.std(signal)),
            f"{prefix}_impulse_ns": float(
                np.trapezoid(signal, dx=1.0 / load_sample_rate)
            ),
            f"{prefix}_abs_impulse_ns": float(
                np.trapezoid(np.abs(signal), dx=1.0 / load_sample_rate)
            ),
            f"{prefix}_dominant_hz": spectrum["dominant_hz"],
            f"{prefix}_band_power_n2": spectrum["band_power"],
            f"{prefix}_spectral_entropy": spectrum["spectral_entropy"],
            f"{prefix}_frequency_resolution_hz": spectrum["frequency_resolution_hz"],
            f"{prefix}_frequency_bin_spacing_hz": spectrum["frequency_bin_spacing_hz"],
            f"{prefix}_welch_nperseg": spectrum["nperseg"],
        })

    command = data["cmd"]
    command_times = data["cmd_t"]
    encoder = data["enc"]
    encoder_times = data["enc_t"]
    if not (command.size and command.ndim == 2 and len(command_times) >= 10):
        result["response_status"] = "no_command_data"
        return result

    common_rate = min(200.0, command_rate if np.isfinite(command_rate) and command_rate > 0 else 50.0,
                      load_sample_rate)
    maximum_frequency = min(maximum_frequency, 0.45 * common_rate)
    grid = np.arange(start, stop, 1.0 / common_rate)
    if len(grid) < 256:
        result["response_status"] = "active_interval_too_short"
        return result

    command_rad = dynamixel_to_radians(repair_servo_zeros(command[:, :2]))
    servo1_command = np.interp(grid, command_times, command_rad[:, 0])
    servo2_command = np.interp(grid, command_times, command_rad[:, 1])

    # Low-pass each force axis before interpolation to the response grid.
    cutoff = min(maximum_frequency * 1.25, common_rate * 0.40)
    sos = butter(6, cutoff, btype="lowpass", fs=load_sample_rate, output="sos")
    active_load_times = sample_times[active]
    force_grids = {
        prefix: np.interp(
            grid, active_load_times, sosfiltfilt(sos, signal)
        )
        for prefix, signal in force_signals.items()
    }

    servo1_spectrum = power_spectrum(servo1_command, common_rate, maximum_frequency)
    servo2_spectrum = power_spectrum(servo2_command, common_rate, maximum_frequency)
    motion_candidates = [
        ("servo1", servo1_command, servo1_spectrum),
        ("servo2", servo2_command, servo2_spectrum),
    ]
    motion_candidates = [item for item in motion_candidates if item[2] is not None]
    if not motion_candidates:
        result["response_status"] = "no_periodic_servo_motion"
        return result
    motion_name, motion_signal, motion_spectrum = max(
        motion_candidates, key=lambda item: item[2]["band_power"]
    )
    observed_frequency = motion_spectrum["dominant_hz"]
    parameters = parameters_for(gait, run)
    gait_frequency = parameters['frequency_hz'] if parameters else observed_frequency
    result.update({
        'observed_motion_peak_hz': observed_frequency,
        'expected_frequency_hz': parameters['frequency_hz'] if parameters else np.nan,
        'frequency_reference': 'parameter_table' if parameters else 'observed_peak_fallback',
        'parameter_source': PARAMETER_SOURCE,
        'parameter_notes': parameters['notes'] if parameters else 'No matching row',
        'parameter_row_assumption': 'table row N = filename run N',
    })

    response_nperseg = min(1024, 2 ** int(np.floor(np.log2(len(grid) // 2))))
    response_nperseg = max(128, response_nperseg)
    result.update({
        "response_status": "ok",
        "dominant_motion_axis": motion_name,
        "gait_frequency_hz": gait_frequency,
        "cycles_analyzed": float((stop - start) * gait_frequency),
        "response_common_rate_hz": common_rate,
    })
    for prefix, force_grid in force_grids.items():
        spectrum = force_spectra[prefix]
        frequencies, coherence_values = coherence(
            detrend(motion_signal), detrend(force_grid), fs=common_rate,
            window="hann", nperseg=response_nperseg,
            noverlap=response_nperseg // 2,
            nfft=max(4096, response_nperseg),
        )
        csd_frequencies, cross = csd(
            detrend(motion_signal), detrend(force_grid), fs=common_rate,
            window="hann", nperseg=response_nperseg,
            noverlap=response_nperseg // 2,
            nfft=max(4096, response_nperseg),
        )
        phase = float(np.angle(
            cross[int(np.argmin(np.abs(csd_frequencies - gait_frequency)))]
        ))
        lag_seconds = (
            -phase / (2.0 * np.pi * gait_frequency)
            if gait_frequency > 0 else np.nan
        )
        frequency_error = (
            100.0 * abs(spectrum["dominant_hz"] - gait_frequency) / gait_frequency
            if gait_frequency > 0 else np.nan
        )
        result.update({
            f"{prefix}_power_at_gait_frequency": value_near_frequency(
                spectrum["frequency"], spectrum["psd"], gait_frequency
            ),
            f"{prefix}_power_at_second_harmonic": value_near_frequency(
                spectrum["frequency"], spectrum["psd"], 2.0 * gait_frequency
            ) if 2.0 * gait_frequency <= maximum_frequency else np.nan,
            f"{prefix}_power_at_third_harmonic": value_near_frequency(
                spectrum["frequency"], spectrum["psd"], 3.0 * gait_frequency
            ) if 3.0 * gait_frequency <= maximum_frequency else np.nan,
            f"motion_{prefix}_coherence": value_near_frequency(
                frequencies, coherence_values, gait_frequency
            ),
            f"motion_to_{prefix}_phase_deg": float(np.degrees(phase)),
            f"motion_to_{prefix}_lag_s": float(lag_seconds),
            f"{prefix}_frequency_error_percent": float(frequency_error),
            f"{prefix}_frequency_match_percent": float(
                np.clip(100.0 - frequency_error, 0.0, 100.0)
            ),
        })

    if encoder.size and encoder.ndim == 2 and len(encoder_times) >= 2:
        encoder_rad = dynamixel_to_radians(repair_servo_zeros(encoder[:, :2]))
        servo1_encoder = np.interp(grid, encoder_times, encoder_rad[:, 0])
        servo2_encoder = np.interp(grid, encoder_times, encoder_rad[:, 1])
        result["servo1_tracking_rmse_rad"] = float(
            np.sqrt(np.mean((servo1_command - servo1_encoder) ** 2))
        )
        result["servo2_tracking_rmse_rad"] = float(
            np.sqrt(np.mean((servo2_command - servo2_encoder) ** 2))
        )
    return result


def write_dict_rows(path, rows):
    """Write heterogeneous dictionaries using the union of their keys."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def scan_global_axes(groups, hampel_seconds, smooth_seconds, hampel_sigma,
                     force_scale_percentile=99.9,
                     load_sample_rate=10_000.0,
                     spectral_max_frequency=50.0):
    """Find common Y-axes from the cleaned signals used in the final plots."""
    force_min, force_max = np.inf, -np.inf
    servo_min, servo_max = np.inf, -np.inf
    errors = []
    diagnostics = []
    spectral_features = []
    cleaned_force_chunks = []

    entries = [
        (flipper, gait, run, path)
        for flipper, gait_map in groups.items()
        for gait, runs in gait_map.items()
        for run, path in runs
    ]
    for index, (flipper, gait, run, bag_path) in enumerate(entries, start=1):
        print(f"Scaling pass {index}/{len(entries)}: {bag_path.name}")
        try:
            data = read_bag(bag_path)
            prepared = prepare_plot_data(
                data, hampel_seconds, smooth_seconds, hampel_sigma,
            )
            diagnostics.extend(
                force_diagnostic_rows(
                    bag_path, flipper, gait, run, data, prepared
                )
            )
            spectral_features.append(analyze_spectrum(
                data, bag_path, flipper, gait, run,
                load_sample_rate=load_sample_rate,
                maximum_frequency=spectral_max_frequency,
                hampel_seconds=hampel_seconds,
                hampel_sigma=hampel_sigma,
            ))
            forces = prepared["forces"]
            net_force = prepared["net_force"]
            plotted_force_values = np.concatenate((
                forces.reshape(-1), net_force.reshape(-1)
            ))
            finite_forces = plotted_force_values[
                np.isfinite(plotted_force_values)
            ]
            if finite_forces.size:
                cleaned_force_chunks.append(
                    np.abs(finite_forces).astype(np.float32, copy=False)
                )
                force_min = min(force_min, float(finite_forces.min()))
                force_max = max(force_max, float(finite_forces.max()))

            for key in ("cmd", "enc"):
                finite_servo = prepared[key][np.isfinite(prepared[key])]
                if finite_servo.size:
                    servo_min = min(servo_min, float(finite_servo.min()))
                    servo_max = max(servo_max, float(finite_servo.max()))

        except Exception as exc:
            errors.append((bag_path.name, str(exc)))
            print(f"ERROR during scaling pass: {bag_path.name}: {exc}")

    if cleaned_force_chunks:
        pooled_absolute_forces = np.concatenate(cleaned_force_chunks)
        robust_force_magnitude = float(np.percentile(
            pooled_absolute_forces, force_scale_percentile
        ))
        robust_force_limits = symmetric_limits(
            -robust_force_magnitude, robust_force_magnitude
        )
    else:
        robust_force_limits = None

    return {
        "force": robust_force_limits,
        "force_observed": symmetric_limits(force_min, force_max),
        "force_scale_percentile": force_scale_percentile,
        "servo": padded_limits(servo_min, servo_max),
    }, errors, diagnostics, spectral_features


def save_run_graph(data, path, flipper, gait, run,
                   hampel_seconds=0.10, smooth_seconds=0.25,
                   hampel_sigma=3.5, global_axes=None):
    fig, axes = plt.subplots(
        2,
        1,
        figsize=(16, 9),
        dpi=160,
        sharex=True,
        gridspec_kw={"height_ratios": [1.12, 1], "hspace": 0.14},
    )
    fig.patch.set_facecolor("white")
    fig.suptitle(
        f"{friendly(flipper)}  |  {friendly(gait)}  |  Run {run}",
        x=0.055,
        y=0.975,
        ha="left",
        fontsize=22,
        fontweight="bold",
        color="#121F35",
    )
    fig.text(
        0.945,
        0.935,
        path.name,
        ha="right",
        va="top",
        fontsize=9,
        color="#68707D",
    )

    fig.text(.065, .902, parameter_summary(parameters_for(gait, run)), fontsize=9)
    prepared = prepare_plot_data(
        data, hampel_seconds, smooth_seconds, hampel_sigma
    )

    forces = prepared["forces"]
    if forces.size:
        t, plotted = downsample(data["load_t"], forces)
        for index in range(3):
            axes[0].plot(
                t,
                plotted[:, index],
                label=LOAD_LABELS[index],
                color=LOAD_COLORS[index],
                linewidth=1.35,
            )
        net_t, net_plotted = downsample(
            prepared["load_t"], prepared["net_force"]
        )
        axes[0].plot(
            net_t,
            net_plotted,
            label="Resultant magnitude",
            color=NET_FORCE_COLOR,
            linestyle=(0, (2.0, 2.0)),
            linewidth=1.8,
            alpha=0.92,
            zorder=4,
        )
    else:
        axes[0].text(
            0.5,
            0.5,
            "No Fx/Fy/Fz load-cell data",
            transform=axes[0].transAxes,
            ha="center",
            va="center",
            fontsize=18,
            color="#68707D",
        )

    for key, time_key, suffix, linestyle in (
        ("cmd", "cmd_t", "Command", "-"),
        ("enc", "enc_t", "Measured", "--"),
    ):
        servo = prepared[key]
        if servo.size:
            t, plotted = downsample(prepared[time_key], servo)
            for index in range(min(2, plotted.shape[1])):
                axes[1].plot(
                    t,
                    counts_to_degrees(plotted[:, index]),
                    label=f"Servo {index + 1} {suffix}",
                    color=SERVO_COLORS[index],
                    linestyle=linestyle,
                    linewidth=1.35,
                )

    axes[0].set_title(
        "Filtered load-cell forces and resultant resultant magnitude",
        loc="left",
        fontsize=16,
        fontweight="bold",
    )
    axes[0].set_ylabel("Force (N)", fontsize=15, fontweight="bold")
    axes[1].set_title(
        "Servo commands and measured positions",
        loc="left",
        fontsize=16,
        fontweight="bold",
    )
    axes[1].set_ylabel(
        "Encoder angle (degrees from encoder zero)", fontsize=15, fontweight="bold"
    )
    axes[1].set_xlabel(
        "Time from bag start (s)", fontsize=15, fontweight="bold"
    )

    if global_axes:
        if global_axes.get("force"):
            axes[0].set_ylim(*global_axes["force"])
            lower, upper = global_axes["force"]
            plotted_force_values = np.concatenate((
                forces.reshape(-1), prepared["net_force"].reshape(-1)
            ))
            finite_forces = plotted_force_values[
                np.isfinite(plotted_force_values)
            ]
            outside = finite_forces[
                (finite_forces < lower) | (finite_forces > upper)
            ]
            if outside.size:
                observed = float(np.max(np.abs(finite_forces)))
                axes[0].text(
                    0.01,
                    0.03,
                    f"Some filtered points are outside the shared graph scale "
                    f"(largest |force|: {observed:.2f} N)",
                    transform=axes[0].transAxes,
                    fontsize=10,
                    color="#9B1C1C",
                    fontweight="bold",
                    va="bottom",
                    bbox={
                        "facecolor": "white",
                        "edgecolor": "#E6A5A5",
                        "alpha": 0.88,
                        "pad": 3,
                    },
                )
        if global_axes.get("servo"):
            axes[1].set_ylim(*counts_to_degrees(global_axes["servo"]))

    for axis in axes:
        axis.grid(True, color="#D9DDE3", linewidth=0.65, alpha=0.9)
        axis.spines[["top", "right"]].set_visible(False)
        axis.spines[["left", "bottom"]].set_color("#AEB5BF")
        handles, labels = axis.get_legend_handles_labels()
        if handles:
            axis.legend(loc="upper right", ncol=6, fontsize=11, frameon=False)
        axis.tick_params(axis="both", which="major", labelsize=12)
        axis.margins(x=0)

    fig.subplots_adjust(left=0.065, right=0.97, top=0.84, bottom=0.075)
    image_path = path.with_suffix(".png")
    fig.savefig(image_path, facecolor="white")
    plt.close(fig)
    return image_path


def set_background(slide, color):
    fill = slide.background.fill
    fill.solid()
    fill.fore_color.rgb = color


def add_text(slide, text, left, top, width, height, size, color, bold=False,
             align=PP_ALIGN.LEFT):
    box = slide.shapes.add_textbox(
        Inches(left), Inches(top), Inches(width), Inches(height)
    )
    frame = box.text_frame
    frame.clear()
    frame.margin_left = frame.margin_right = 0
    frame.margin_top = frame.margin_bottom = 0
    paragraph = frame.paragraphs[0]
    paragraph.text = text
    paragraph.alignment = align
    run = paragraph.runs[0]
    run.font.name = "Arial"
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.color.rgb = color
    return box


def add_title_slide(prs, total_bags, group_count):
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_background(slide, NAVY)
    add_text(
        slide,
        "CRAB FLIPPER TESTING",
        0.65,
        0.45,
        5.5,
        0.35,
        15,
        LIGHT_BLUE,
        True,
    )
    add_text(
        slide,
        "Flipper performance analysis",
        0.65,
        2.15,
        11.8,
        0.85,
        38,
        WHITE,
        True,
    )
    add_text(
        slide,
        f"{total_bags} ROS bags • {len(FLIPPERS)} flipper types • "
        f"{group_count} gait families",
        0.65,
        3.15,
        11.8,
        0.45,
        20,
        WHITE,
    )
    add_text(
        slide,
        "Main comparisons first • Every individual run is included in the appendix",
        0.65,
        5.95,
        11.8,
        0.4,
        17,
        LIGHT_BLUE,
    )


def add_section_slide(prs, label, eyebrow, detail):
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_background(slide, WHITE)
    add_text(slide, eyebrow.upper(), 0.65, 0.5, 8.5, 0.35, 14, BLUE, True)
    add_text(slide, label, 0.65, 2.2, 11.8, 0.85, 38, NAVY, True)
    add_text(slide, detail, 0.65, 3.25, 11.8, 0.5, 19, GRAY)
    line = slide.shapes.add_shape(
        1,
        Inches(0.65),
        Inches(5.95),
        Inches(12.0),
        Inches(0.08),
    )
    line.fill.solid()
    line.fill.fore_color.rgb = LIGHT_BLUE
    line.line.fill.background()


def add_graph_slide(prs, image_path):
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    slide.shapes.add_picture(
        str(image_path), 0, 0, width=SLIDE_W, height=SLIDE_H
    )


def summary_records(diagnostics, allowed_bags=None):
    """Aggregate average absolute force by flipper, gait, and axis."""
    grouped = defaultdict(list)
    for row in diagnostics:
        if allowed_bags is not None and row.get("bag") not in allowed_bags:
            continue
        grouped[(row["flipper"], row["gait"], row["axis"])].append(
            float(row["clean_abs_mean"])
        )

    records = []
    for (flipper, gait, axis), values in grouped.items():
        values = np.asarray(values, dtype=np.float64)
        records.append({
            "flipper": flipper,
            "gait": gait,
            "axis": axis,
            "runs": len(values),
            "median": float(np.median(values)),
            "q1": float(np.percentile(values, 25)),
            "q3": float(np.percentile(values, 75)),
        })
    return records


def save_coverage_summary(groups, output_path):
    """Create a run-count matrix for the opening analysis section."""
    flippers = [f for f in FLIPPERS if groups.get(f)]
    gaits = sorted(
        {gait for gait_map in groups.values() for gait in gait_map},
        key=gait_sort_key,
    )
    matrix = np.zeros((len(flippers), len(gaits)), dtype=int)
    for row, flipper in enumerate(flippers):
        for column, gait in enumerate(gaits):
            matrix[row, column] = len(groups[flipper].get(gait, []))

    fig, axis = plt.subplots(figsize=(16, 9), dpi=160)
    image = axis.imshow(matrix, cmap="Blues", vmin=0, aspect="auto")
    axis.set_xticks(range(len(gaits)), [compact_gait_name(g) for g in gaits],
                    rotation=32, ha="right", fontsize=11)
    axis.set_yticks(range(len(flippers)), [friendly(f) for f in flippers],
                    fontsize=13)
    axis.set_xlabel("Gait family", fontsize=14, fontweight="bold")
    axis.set_ylabel("Flipper", fontsize=14, fontweight="bold")
    fig.suptitle(
        "What was tested?",
        x=0.075, y=0.975, ha="left", fontsize=25,
        fontweight="bold", color="#121F35",
    )
    fig.text(
        0.075, 0.925,
        "Numbers show how many bags were recorded. A dash means that "
        "combination was not tested.",
        fontsize=13, color="#5C6572",
    )
    for row in range(matrix.shape[0]):
        for column in range(matrix.shape[1]):
            value = matrix[row, column]
            if value:
                axis.text(
                    column, row, str(value), ha="center", va="center",
                    fontsize=14, fontweight="bold",
                    color="white" if value > matrix.max() * 0.52 else "#121F35",
                )
            else:
                axis.text(column, row, "—", ha="center", va="center",
                          fontsize=12, color="#98A1AD")
    axis.set_xticks(np.arange(-0.5, len(gaits), 1), minor=True)
    axis.set_yticks(np.arange(-0.5, len(flippers), 1), minor=True)
    axis.grid(which="minor", color="white", linewidth=2)
    axis.tick_params(which="minor", bottom=False, left=False)
    for spine in axis.spines.values():
        spine.set_visible(False)
    colorbar = fig.colorbar(image, ax=axis, fraction=0.025, pad=0.02)
    colorbar.set_label("Recorded runs", fontsize=12)
    colorbar.ax.tick_params(labelsize=11)
    fig.subplots_adjust(left=0.12, right=0.93, top=0.83, bottom=0.22)
    fig.savefig(output_path, facecolor="white")
    plt.close(fig)
    return output_path


def save_sustained_force_summary(groups, diagnostics, output_path):
    """Create Fx/Fy/Fz heatmaps using average absolute force."""
    records = summary_records(diagnostics)
    flippers = [f for f in FLIPPERS if groups.get(f)]
    gaits = sorted(
        {gait for gait_map in groups.values() for gait in gait_map},
        key=natural_key,
    )
    lookup = {
        (r["flipper"], r["gait"], r["axis"]): r["median"] for r in records
    }
    matrices = []
    for axis_name in LOAD_LABELS:
        matrix = np.full((len(flippers), len(gaits)), np.nan)
        for row, flipper in enumerate(flippers):
            for column, gait in enumerate(gaits):
                value = lookup.get((flipper, gait, axis_name))
                if value is not None:
                    matrix[row, column] = value
        matrices.append(matrix)

    finite = np.concatenate([m[np.isfinite(m)] for m in matrices])
    vmax = float(np.max(finite)) if finite.size else 1.0
    cmap = plt.cm.YlGnBu.copy()
    cmap.set_bad("#EEF1F5")
    fig, axes = plt.subplots(3, 1, figsize=(16, 9), dpi=160)
    fig.suptitle(
        "Average force magnitude for each flipper and gait",
        x=0.065, y=0.975, ha="left", fontsize=24,
        fontweight="bold", color="#121F35",
    )
    fig.text(
        0.065, 0.925,
        "We make every force value positive, average each run, and then take "
        "the middle result across its runs.",
        fontsize=12.5, color="#5C6572",
    )
    last_image = None
    for index, (axis, matrix, axis_name) in enumerate(
        zip(axes, matrices, LOAD_LABELS)
    ):
        last_image = axis.imshow(matrix, cmap=cmap, vmin=0, vmax=vmax,
                                 aspect="auto")
        axis.set_yticks(range(len(flippers)), [friendly(f) for f in flippers],
                        fontsize=10)
        axis.set_ylabel(axis_name, fontsize=14, fontweight="bold",
                        rotation=0, labelpad=28, va="center")
        if index == len(axes) - 1:
            axis.set_xticks(
                range(len(gaits)), [compact_gait_name(g) for g in gaits],
                            rotation=27, ha="right", fontsize=9)
        else:
            axis.set_xticks(range(len(gaits)), [])
        for row in range(matrix.shape[0]):
            for column in range(matrix.shape[1]):
                value = matrix[row, column]
                label = "—" if not np.isfinite(value) else f"{value:.1f}"
                color = "#98A1AD" if not np.isfinite(value) else (
                    "white" if value > vmax * 0.58 else "#121F35"
                )
                axis.text(column, row, label, ha="center", va="center",
                          fontsize=9.5, fontweight="bold", color=color)
        axis.set_xticks(np.arange(-0.5, len(gaits), 1), minor=True)
        axis.set_yticks(np.arange(-0.5, len(flippers), 1), minor=True)
        axis.grid(which="minor", color="white", linewidth=1.5)
        axis.tick_params(which="minor", bottom=False, left=False)
        for spine in axis.spines.values():
            spine.set_visible(False)
    if last_image is not None:
        colorbar_axis = fig.add_axes([0.94, 0.27, 0.014, 0.46])
        colorbar = fig.colorbar(last_image, cax=colorbar_axis)
        colorbar.set_label("Average |force| (N)", fontsize=12)
        colorbar.ax.tick_params(labelsize=10)
    fig.subplots_adjust(left=0.12, right=0.91, top=0.86, bottom=0.20, hspace=0.22)
    fig.savefig(output_path, facecolor="white")
    plt.close(fig)
    return output_path


def save_top_performers(diagnostics, output_path, top_count=4):
    """Show the largest average absolute forces by axis."""
    records = summary_records(diagnostics)
    fig, axes = plt.subplots(1, 3, figsize=(16, 9), dpi=160)
    fig.suptitle(
        "Which combinations produced the largest average forces?",
        x=0.055, y=0.975, ha="left", fontsize=24,
        fontweight="bold", color="#121F35",
    )
    fig.text(
        0.055, 0.925,
        "Longer bars mean larger average |force|. The thin lines show how much "
        "the results changed across runs.",
        fontsize=12.5, color="#5C6572",
    )
    for axis, axis_name, color in zip(axes, LOAD_LABELS, LOAD_COLORS):
        ranked = sorted(
            (r for r in records if r["axis"] == axis_name),
            key=lambda r: r["median"], reverse=True,
        )[:top_count]
        ranked.reverse()
        labels = [
            f"{friendly(r['flipper'])}\n{compact_gait_name(r['gait'])}"
            for r in ranked
        ]
        medians = np.asarray([r["median"] for r in ranked])
        lower = medians - np.asarray([r["q1"] for r in ranked])
        upper = np.asarray([r["q3"] for r in ranked]) - medians
        positions = np.arange(len(ranked))
        axis.barh(positions, medians, color=color, alpha=0.86)
        axis.errorbar(
            medians, positions, xerr=np.vstack([lower, upper]), fmt="none",
            ecolor="#121F35", elinewidth=1.2, capsize=3,
        )
        axis.set_yticks(positions, labels, fontsize=9.5)
        axis.set_title(axis_name, fontsize=17, fontweight="bold")
        axis.set_xlabel("Average |force| (N)", fontsize=11, fontweight="bold")
        axis.tick_params(axis="x", labelsize=10)
        axis.grid(axis="x", color="#D9DDE3", linewidth=0.7)
        axis.spines[["top", "right", "left"]].set_visible(False)
        for position, value in zip(positions, medians):
            axis.text(
                value * 0.97, position, f"{value:.2f}", ha="right",
                va="center", fontsize=9.5, fontweight="bold", color="white",
            )
        if ranked:
            axis.set_xlim(0, max(r["q3"] for r in ranked) * 1.10)
    fig.subplots_adjust(left=0.13, right=0.98, top=0.84, bottom=0.10, wspace=0.55)
    fig.savefig(output_path, facecolor="white")
    plt.close(fig)
    return output_path


def valid_spectral_rows(rows, minimum_cycles=3.0):
    """Keep runs with enough observed cycles for summary-level spectra."""
    valid = []
    for row in rows:
        try:
            cycles = float(row.get("cycles_analyzed", np.nan))
        except (TypeError, ValueError):
            continue
        if (
            row.get("status") == "ok"
            and row.get("response_status") == "ok"
            and np.isfinite(cycles)
            and cycles >= minimum_cycles
        ):
            valid.append(row)
    return valid


def aggregate_spectral_rows(rows):
    """Median spectral metrics by matched flipper and gait labels."""
    grouped = defaultdict(list)
    for row in valid_spectral_rows(rows):
        grouped[(row["flipper"], row["gait"])].append(row)

    output = []
    metrics = (
        "gait_frequency_hz", "fy_dominant_hz", "motion_fy_coherence",
        "fy_mean_n", "fy_mean_abs_n", "fy_rms_n", "fy_std_n",
        "fy_spectral_entropy",
    )
    for (flipper, gait), group in grouped.items():
        record = {"flipper": flipper, "gait": gait, "runs": len(group)}
        for metric in metrics:
            values = []
            for row in group:
                try:
                    value = float(row.get(metric, np.nan))
                except (TypeError, ValueError):
                    continue
                if np.isfinite(value):
                    values.append(value)
            record[metric] = float(np.median(values)) if values else np.nan
        for harmonic_name, output_name in (
            ("fy_power_at_second_harmonic", "second_harmonic_ratio"),
            ("fy_power_at_third_harmonic", "third_harmonic_ratio"),
        ):
            ratios = []
            for row in group:
                try:
                    fundamental = float(row.get("fy_power_at_gait_frequency", np.nan))
                    harmonic = float(row.get(harmonic_name, np.nan))
                except (TypeError, ValueError):
                    continue
                if np.isfinite(fundamental) and fundamental > 0 and np.isfinite(harmonic):
                    ratios.append(max(harmonic, 0.0) / fundamental)
            record[output_name] = float(np.median(ratios)) if ratios else np.nan
        gait_frequency = record["gait_frequency_hz"]
        force_frequency = record["fy_dominant_hz"]
        record["frequency_error_percent"] = (
            100.0 * abs(force_frequency - gait_frequency) / gait_frequency
            if np.isfinite(gait_frequency) and gait_frequency > 0
            else np.nan
        )
        output.append(record)
    return output


def aggregate_axis_spectral_rows(rows):
    """Aggregate frequency matching separately for Fx, Fy, and Fz."""
    grouped = defaultdict(list)
    for row in valid_spectral_rows(rows):
        for axis_name in LOAD_LABELS:
            prefix = axis_name.lower()
            try:
                dominant = float(row.get(f"{prefix}_dominant_hz", np.nan))
                gait_frequency = float(row.get("gait_frequency_hz", np.nan))
            except (TypeError, ValueError):
                continue
            if not (
                np.isfinite(dominant) and np.isfinite(gait_frequency)
                and gait_frequency > 0
            ):
                continue
            grouped[(row["flipper"], row["gait"], axis_name)].append(row)

    output = []
    for (flipper, gait, axis_name), group in grouped.items():
        prefix = axis_name.lower()
        record = {
            "flipper": flipper,
            "gait": gait,
            "axis": axis_name,
            "runs": len(group),
        }
        for metric in (
            f"{prefix}_dominant_hz",
            f"motion_{prefix}_coherence",
            f"{prefix}_mean_abs_n",
            f"{prefix}_std_n",
        ):
            values = []
            for row in group:
                try:
                    value = float(row.get(metric, np.nan))
                except (TypeError, ValueError):
                    continue
                if np.isfinite(value):
                    values.append(value)
            record[metric] = float(np.median(values)) if values else np.nan

        errors = []
        matches = []
        for row in group:
            try:
                error = float(row.get(
                    f"{prefix}_frequency_error_percent", np.nan
                ))
            except (TypeError, ValueError):
                error = np.nan
            if not np.isfinite(error):
                dominant = float(row[f"{prefix}_dominant_hz"])
                gait_frequency = float(row["gait_frequency_hz"])
                error = 100.0 * abs(dominant - gait_frequency) / gait_frequency
            errors.append(error)
            matches.append(float(np.clip(100.0 - error, 0.0, 100.0)))
        record["frequency_error_percent"] = float(np.median(errors))
        record["frequency_match_percent"] = float(np.median(matches))
        output.append(record)
    return output


def aggregate_force_spectral_rows(rows, force_axis="Fy"):
    """Aggregate one force axis and measure variation across repeated runs."""
    prefix = force_axis.lower()
    grouped = defaultdict(list)
    for row in valid_spectral_rows(rows):
        try:
            dominant = float(row.get(f"{prefix}_dominant_hz", np.nan))
        except (TypeError, ValueError):
            continue
        if np.isfinite(dominant):
            grouped[(row["flipper"], row["gait"])].append(row)

    output = []
    source_metrics = {
        "gait_frequency_hz": "gait_frequency_hz",
        "dominant_hz": f"{prefix}_dominant_hz",
        "motion_coherence": f"motion_{prefix}_coherence",
        "within_run_std_n": f"{prefix}_std_n",
        "spectral_entropy": f"{prefix}_spectral_entropy",
    }
    for (flipper, gait), group in grouped.items():
        record = {
            "flipper": flipper,
            "gait": gait,
            "axis": force_axis,
            "runs": len(group),
        }
        for output_name, source_name in source_metrics.items():
            values = []
            for row in group:
                try:
                    value = float(row.get(source_name, np.nan))
                except (TypeError, ValueError):
                    continue
                if np.isfinite(value):
                    values.append(value)
            record[output_name] = float(np.median(values)) if values else np.nan

        run_average_forces = []
        for row in group:
            try:
                value = float(row.get(f"{prefix}_mean_abs_n", np.nan))
            except (TypeError, ValueError):
                continue
            if np.isfinite(value):
                run_average_forces.append(value)
        record["runs"] = len(run_average_forces)
        record["mean_abs_n"] = (
            float(np.mean(run_average_forces))
            if run_average_forces else np.nan
        )
        # The comparison slides use between-run repeatability, not the spread
        # of samples inside one run. With one run, this quantity is undefined.
        record["std_n"] = (
            float(np.std(run_average_forces, ddof=1))
            if len(run_average_forces) >= 2 else np.nan
        )

        for order, output_name in (
            ("second", "second_harmonic_ratio"),
            ("third", "third_harmonic_ratio"),
        ):
            ratios = []
            for row in group:
                try:
                    fundamental = float(row.get(
                        f"{prefix}_power_at_gait_frequency", np.nan
                    ))
                    harmonic = float(row.get(
                        f"{prefix}_power_at_{order}_harmonic", np.nan
                    ))
                except (TypeError, ValueError):
                    continue
                if (
                    np.isfinite(fundamental) and fundamental > 0
                    and np.isfinite(harmonic)
                ):
                    ratios.append(max(harmonic, 0.0) / fundamental)
            record[output_name] = float(np.median(ratios)) if ratios else np.nan
        output.append(record)
    return output


def aggregate_gait_spectral_rows(rows, force_axis="Fy"):
    """Give each gait one vote by first aggregating within flipper–gait."""
    grouped = defaultdict(list)
    for record in aggregate_force_spectral_rows(rows, force_axis):
        grouped[record["gait"]].append(record)

    metrics = (
        "gait_frequency_hz", "second_harmonic_ratio",
        "third_harmonic_ratio", "spectral_entropy",
    )
    output = []
    for gait, group in grouped.items():
        record = {
            "gait": gait,
            "flipper_count": len(group),
            "run_count": sum(item["runs"] for item in group),
        }
        for metric in metrics:
            values = [
                item[metric] for item in group
                if np.isfinite(item.get(metric, np.nan))
            ]
            record[metric] = float(np.median(values)) if values else np.nan
        output.append(record)
    return sorted(output, key=lambda item: gait_sort_key(item["gait"]))


def silhouette_score_from_labels(values, labels):
    """Compute a small-sample silhouette score without extra dependencies."""
    distances = np.linalg.norm(
        values[:, np.newaxis, :] - values[np.newaxis, :, :], axis=2
    )
    scores = []
    for index, label in enumerate(labels):
        same = np.flatnonzero(labels == label)
        same = same[same != index]
        if same.size == 0:
            scores.append(0.0)
            continue
        within = float(np.mean(distances[index, same]))
        between = min(
            float(np.mean(distances[index, labels == other]))
            for other in np.unique(labels) if other != label
        )
        denominator = max(within, between)
        scores.append((between - within) / denominator if denominator else 0.0)
    return float(np.mean(scores))


def deterministic_cluster_labels(values, minimum_clusters=2, maximum_clusters=4):
    """Choose 2–4 clusters using deterministic initialization and silhouette."""
    best = None
    maximum_clusters = min(maximum_clusters, len(values) - 1)
    for cluster_count in range(minimum_clusters, maximum_clusters + 1):
        mean = np.mean(values, axis=0)
        center_indices = [int(np.argmax(np.linalg.norm(values - mean, axis=1)))]
        while len(center_indices) < cluster_count:
            distance_to_centers = np.min(
                np.linalg.norm(
                    values[:, np.newaxis, :] - values[center_indices][np.newaxis, :, :],
                    axis=2,
                ),
                axis=1,
            )
            distance_to_centers[center_indices] = -np.inf
            center_indices.append(int(np.argmax(distance_to_centers)))
        centers, labels = kmeans2(
            values, values[center_indices], iter=100, minit="matrix",
            missing="raise", check_finite=True,
        )
        labels = np.asarray(labels, dtype=int)
        counts = np.bincount(labels, minlength=cluster_count)
        if np.any(counts < 2):
            continue
        score = silhouette_score_from_labels(values, labels)
        if best is None or score > best[0]:
            best = (score, labels, centers)
    if best is None:
        return np.zeros(len(values), dtype=int), np.mean(values, axis=0)[None, :], np.nan
    score, labels, centers = best
    return labels, centers, score


def spectral_similarity_projection(rows, force_axis="Fy"):
    """Project gait spectra into two dimensions and find exploratory clusters."""
    records = aggregate_gait_spectral_rows(rows, force_axis)
    usable = [
        record for record in records
        if all(np.isfinite(record[metric]) for metric in (
            "gait_frequency_hz", "second_harmonic_ratio",
            "third_harmonic_ratio", "spectral_entropy",
        ))
    ]
    if len(usable) < 4:
        return usable, np.empty((len(usable), 2)), np.zeros(len(usable), dtype=int), np.nan, np.zeros(2)

    raw = np.asarray([
        [
            np.log10(max(record["gait_frequency_hz"], 1e-9)),
            np.log10(1.0 + max(record["second_harmonic_ratio"], 0.0)),
            np.log10(1.0 + max(record["third_harmonic_ratio"], 0.0)),
            record["spectral_entropy"],
        ]
        for record in usable
    ])
    median = np.median(raw, axis=0)
    scale = np.percentile(raw, 75, axis=0) - np.percentile(raw, 25, axis=0)
    fallback = np.std(raw, axis=0)
    scale = np.where(scale > 1e-12, scale, fallback)
    scale = np.where(scale > 1e-12, scale, 1.0)
    standardized = (raw - median) / scale

    centered = standardized - np.mean(standardized, axis=0)
    _, singular_values, components = np.linalg.svd(centered, full_matrices=False)
    coordinates = centered @ components[:2].T
    variance = singular_values ** 2
    explained = variance[:2] / np.sum(variance) if np.sum(variance) else np.zeros(2)

    # Keep higher-harmonic responses on the right and higher frequencies above.
    harmonic_score = raw[:, 1] + raw[:, 2]
    if np.corrcoef(coordinates[:, 0], harmonic_score)[0, 1] < 0:
        coordinates[:, 0] *= -1
    if np.corrcoef(coordinates[:, 1], raw[:, 0])[0, 1] < 0:
        coordinates[:, 1] *= -1

    labels, _, score = deterministic_cluster_labels(standardized)
    harmonic_medians = {
        label: float(np.median(harmonic_score[labels == label]))
        for label in np.unique(labels)
    }
    ordered = sorted(harmonic_medians, key=harmonic_medians.get)
    remap = {old: new for new, old in enumerate(ordered)}
    labels = np.asarray([remap[label] for label in labels], dtype=int)
    return usable, coordinates, labels, score, explained


def draw_similarity_panel(axis, spectral_rows, force_axis):
    """Draw one compact gait-similarity map inside a multi-panel slide."""
    records, coordinates, labels, _, explained = spectral_similarity_projection(
        spectral_rows, force_axis
    )
    palette = ("#2F80ED", "#27AE60", "#F2994A", "#9B51E0", "#EB5757")
    cluster_count = len(np.unique(labels)) if len(labels) else 0
    if not records:
        axis.text(
            0.5, 0.5, f"Not enough {force_axis} data",
            transform=axis.transAxes, ha="center", va="center",
            fontsize=15, color="#68707D",
        )
    else:
        for cluster in range(cluster_count):
            selected = labels == cluster
            color = palette[cluster % len(palette)]
            sizes = [
                80 + 28 * records[index]["flipper_count"]
                for index in np.flatnonzero(selected)
            ]
            axis.scatter(
                coordinates[selected, 0], coordinates[selected, 1],
                s=sizes, color=color, alpha=0.88, edgecolor="white",
                linewidth=1.1, label=f"Group {cluster + 1}", zorder=3,
            )
            centroid = np.mean(coordinates[selected], axis=0)
            axis.scatter(
                [centroid[0]], [centroid[1]], s=430, color=color,
                alpha=0.10, edgecolor="none", zorder=1,
            )

        center = np.mean(coordinates, axis=0)
        for index, (record, point) in enumerate(zip(records, coordinates)):
            right = point[0] >= center[0]
            vertical = 7 if index % 2 == 0 else -10
            axis.annotate(
                compact_gait_name(record["gait"]), point,
                xytext=(6 if right else -6, vertical),
                textcoords="offset points", ha="left" if right else "right",
                va="bottom" if vertical > 0 else "top",
                fontsize=7.8, fontweight="semibold", color="#121F35",
                zorder=4,
            )

    axis.axhline(0, color="#CBD2DB", linewidth=0.7, zorder=0)
    axis.axvline(0, color="#CBD2DB", linewidth=0.7, zorder=0)
    axis.set_title(f"{force_axis} patterns", fontsize=17, fontweight="bold")
    axis.set_xlabel(
        f"Similarity 1 ({explained[0] * 100:.0f}%)",
        fontsize=11.5, fontweight="bold",
    )
    axis.set_ylabel(
        f"Similarity 2 ({explained[1] * 100:.0f}%)",
        fontsize=11.5, fontweight="bold",
    )
    axis.tick_params(labelsize=9)
    axis.grid(True, color="#E1E5EA", linewidth=0.65)
    axis.spines[["top", "right"]].set_visible(False)
    axis.margins(x=0.14, y=0.16)
    if cluster_count:
        axis.legend(loc="best", frameon=False, fontsize=8.5, ncol=2)


def add_similarity_equation(fig, left=0.065):
    """Explain the exact feature-space distance shown by similarity maps."""
    fig.text(
        left, 0.155,
        "Similarity calculation",
        fontsize=10.2, fontweight="bold", color="#121F35",
    )
    fig.text(
        left, 0.126,
        r"$\mathbf{x}=[\log_{10}f_g,\ \log_{10}(1+P_{2g}/P_g),\ "
        r"\log_{10}(1+P_{3g}/P_g),\ H]$",
        fontsize=10.2, color="#314A6E",
    )
    fig.text(
        left, 0.096,
        r"$H=-\sum_k p_k\ln(p_k)/\ln(N),\quad p_k=PSD_k/\sum_j PSD_j,\quad "
        r"\mathbf{z}=(\mathbf{x}-\mathrm{median}(\mathbf{x}))/IQR(\mathbf{x})$",
        fontsize=9.5, color="#314A6E",
    )
    fig.text(
        left, 0.065,
        r"$d(i,j)=\|\mathbf{z}_i-\mathbf{z}_j\|_2$ (smaller = more similar),  "
        r"map $=PCA_{1,2}(\mathbf{z})$,  "
        r"group $=\arg\min_c\|\mathbf{z}-\boldsymbol{\mu}_c\|_2^2$",
        fontsize=9.5, color="#314A6E",
    )


def save_other_axis_similarity_maps(spectral_rows, output_path):
    """Put the Fx and Fz gait-similarity maps on one slide."""
    fig, axes = plt.subplots(1, 2, figsize=(16, 9), dpi=160)
    fig.suptitle(
        "Fx and Fz reveal additional gait-pattern groups",
        x=0.065, y=0.97, ha="left", fontsize=24,
        fontweight="bold", color="#121F35",
    )
    fig.text(
        0.065, 0.915,
        "Gaits that appear close together create similar repeating force "
        "patterns. Colors are calculated separately for each force axis.",
        fontsize=12.2, color="#5C6572",
    )
    for axis, force_axis in zip(axes, ("Fx", "Fz")):
        draw_similarity_panel(axis, spectral_rows, force_axis)
    add_similarity_equation(fig)
    fig.text(
        0.065, 0.025,
        "Pg, P2g, and P3g are the PSD at the gait frequency and its second and third harmonics. Groups describe similarity, not performance.",
        fontsize=9.5, color="#68707D",
    )
    fig.subplots_adjust(
        left=0.08, right=0.98, top=0.82, bottom=0.23, wspace=0.24
    )
    fig.savefig(output_path, facecolor="white")
    plt.close(fig)
    return output_path


def save_spectral_summary(groups, spectral_rows, output_path):
    """Show gait-frequency matching for Fx, Fy, and Fz."""
    records = aggregate_axis_spectral_rows(spectral_rows)
    flippers = [f for f in FLIPPERS if groups.get(f)]
    gaits = sorted(
        {gait for gait_map in groups.values() for gait in gait_map},
        key=natural_key,
    )
    lookup = {
        (r["flipper"], r["gait"], r["axis"]): r["frequency_match_percent"]
        for r in records
    }
    fig, axes = plt.subplots(3, 1, figsize=(16, 9), dpi=160)
    fig.suptitle(
        "How closely does each force repeat with the gait?",
        x=0.065, y=0.975, ha="left", fontsize=24,
        fontweight="bold", color="#121F35",
    )
    excluded = len(spectral_rows) - len(valid_spectral_rows(spectral_rows))
    fig.text(
        0.065, 0.925,
        "Score = max(0, 100 - 100|f_force - f_expected|/f_expected). "
        f"{excluded} short runs with fewer than three cycles are left out.",
        fontsize=12.2, color="#5C6572",
    )

    cmap = plt.colormaps["YlGnBu"].copy()
    cmap.set_bad("#EEF1F5")
    last_image = None
    for index, (axis, axis_name) in enumerate(zip(axes, LOAD_LABELS)):
        matrix = np.full((len(flippers), len(gaits)), np.nan)
        for row_index, flipper in enumerate(flippers):
            for column_index, gait in enumerate(gaits):
                value = lookup.get((flipper, gait, axis_name))
                if value is not None:
                    matrix[row_index, column_index] = value
        last_image = axis.imshow(
            matrix, cmap=cmap, vmin=0.0, vmax=100.0, aspect="auto"
        )
        axis.set_yticks(range(len(flippers)), [friendly(f) for f in flippers], fontsize=9.5)
        axis.set_title(axis_name, loc="left", fontsize=13, fontweight="bold", pad=5)
        if index == 2:
            axis.set_xticks(
                range(len(gaits)), [compact_gait_name(g) for g in gaits],
                            rotation=27, ha="right", fontsize=9)
        else:
            axis.set_xticks(range(len(gaits)), [])
        for row_index in range(matrix.shape[0]):
            for column_index in range(matrix.shape[1]):
                value = matrix[row_index, column_index]
                if not np.isfinite(value):
                    label, text_color = "—", "#98A1AD"
                else:
                    label = f"{value:.0f}%"
                    text_color = "white" if value > 60.0 else "#121F35"
                axis.text(column_index, row_index, label, ha="center", va="center",
                          fontsize=8.8, fontweight="bold", color=text_color)
        axis.set_xticks(np.arange(-0.5, len(gaits), 1), minor=True)
        axis.set_yticks(np.arange(-0.5, len(flippers), 1), minor=True)
        axis.grid(which="minor", color="white", linewidth=1.4)
        axis.tick_params(which="minor", bottom=False, left=False)
        for spine in axis.spines.values():
            spine.set_visible(False)
    if last_image is not None:
        colorbar_axis = fig.add_axes([0.94, 0.27, 0.014, 0.46])
        colorbar = fig.colorbar(last_image, cax=colorbar_axis)
        colorbar.set_label("Frequency match (%)", fontsize=11)
        colorbar.ax.tick_params(labelsize=9)
    fig.subplots_adjust(left=0.20, right=0.91, top=0.86, bottom=0.20, hspace=0.24)
    fig.savefig(output_path, facecolor="white")
    plt.close(fig)
    return output_path


def save_combined_force_frequency_summary(diagnostics, spectral_rows, output_path):
    """Combine average force magnitude and gait-frequency matching."""
    valid_bags = {row["bag"] for row in valid_spectral_rows(spectral_rows)}
    average_records = summary_records(diagnostics, allowed_bags=valid_bags)
    average_lookup = {
        (record["flipper"], record["gait"], record["axis"]): record["median"]
        for record in average_records
    }
    records = []
    for record in aggregate_axis_spectral_rows(spectral_rows):
        average_force = average_lookup.get(
            (record["flipper"], record["gait"], record["axis"])
        )
        if average_force is None or not np.isfinite(average_force):
            continue
        records.append({**record, "average_force_n": average_force})

    fig, axis = plt.subplots(figsize=(16, 9), dpi=160)
    fig.suptitle(
        "Force magnitude and frequency agreement",
        x=0.07, y=0.97, ha="left", fontsize=24,
        fontweight="bold", color="#121F35",
    )
    fig.text(
        0.07, 0.915,
        "Upper-right is the clearest combination: larger average |force| and "
        "a closer frequency match. This is not a measure of thrust efficiency.",
        fontsize=12.2, color="#5C6572",
    )
    axis.axvspan(90, 100, color="#EAF7EE", alpha=0.85, zorder=0)
    for axis_name, color in zip(LOAD_LABELS, LOAD_COLORS):
        subset = [record for record in records if record["axis"] == axis_name]
        if not subset:
            continue
        axis.scatter(
            [record["frequency_match_percent"] for record in subset],
            [record["average_force_n"] for record in subset],
            s=145, color=color, alpha=0.76, edgecolor="white",
            linewidth=1.2, label=axis_name, zorder=3,
        )
        best = max(
            subset,
            key=lambda record: (
                record["average_force_n"]
                * record["frequency_match_percent"] / 100.0
            ),
        )
        right_side = best["frequency_match_percent"] >= 75.0
        axis.annotate(
            f"{axis_name}: {friendly(best['flipper'])} / {friendly(best['gait'])}",
            (best["frequency_match_percent"], best["average_force_n"]),
            xytext=(-8 if right_side else 8, 8),
            textcoords="offset points",
            ha="right" if right_side else "left", va="bottom",
            fontsize=9.5, fontweight="semibold", color="#121F35",
        )

    axis.text(
        0.985, 0.965, "Stronger + better matched",
        transform=axis.transAxes, ha="right", va="top",
        fontsize=11, fontweight="bold", color="#237A45",
    )
    axis.set_xlim(0, 102)
    if records:
        axis.set_ylim(0, max(record["average_force_n"] for record in records) * 1.16)
    axis.set_xlabel("Frequency match with gait (%)", fontsize=15, fontweight="bold")
    axis.set_ylabel("Average |force| (N)", fontsize=15, fontweight="bold")
    axis.tick_params(labelsize=12)
    axis.grid(True, color="#D9DDE3", linewidth=0.75)
    axis.spines[["top", "right"]].set_visible(False)
    axis.legend(loc="upper left", frameon=False, fontsize=12, ncol=3)
    fig.text(
        0.07, 0.055,
        "Each dot is one flipper–gait combination. Color identifies Fx, Fy, or Fz.",
        fontsize=10.5, color="#68707D",
    )
    fig.subplots_adjust(left=0.10, right=0.97, top=0.84, bottom=0.14)
    fig.savefig(output_path, facecolor="white")
    plt.close(fig)
    return output_path


def save_spectral_similarity_map(spectral_rows, output_path, force_axis="Fy"):
    """Place all gait families on one force-axis frequency-domain map."""
    records, coordinates, labels, score, explained = spectral_similarity_projection(
        spectral_rows, force_axis
    )
    cluster_count = len(np.unique(labels)) if len(labels) else 0
    palette = ("#2F80ED", "#27AE60", "#F2994A", "#9B51E0", "#EB5757")

    fig, axis = plt.subplots(figsize=(16, 9), dpi=160)
    force_name = "net-force" if force_axis.lower() == "net" else force_axis
    fig.suptitle(
        f"Gaits with similar {force_name} patterns appear close together",
        x=0.07, y=0.97, ha="left", fontsize=24,
        fontweight="bold", color="#121F35",
    )
    fig.text(
        0.07, 0.915,
        f"Each dot is one gait. Position comes from its {force_name} main "
        "frequency, repeated frequency patterns, and how spread out the signal is.",
        fontsize=12.2, color="#5C6572",
    )

    if not records:
        axis.text(
            0.5, 0.5,
            f"Not enough valid {force_name} spectral data for clustering",
            transform=axis.transAxes, ha="center", va="center",
            fontsize=18, color="#68707D",
        )
    else:
        for cluster in range(cluster_count):
            selected = labels == cluster
            color = palette[cluster % len(palette)]
            sizes = [
                170 + 55 * records[index]["flipper_count"]
                for index in np.flatnonzero(selected)
            ]
            axis.scatter(
                coordinates[selected, 0], coordinates[selected, 1],
                s=sizes, color=color, alpha=0.88, edgecolor="white",
                linewidth=1.5, label=f"Cluster {cluster + 1}", zorder=3,
            )
            centroid = np.mean(coordinates[selected], axis=0)
            axis.scatter(
                [centroid[0]], [centroid[1]], s=900, color=color,
                alpha=0.10, edgecolor="none", zorder=1,
            )

        center = np.mean(coordinates, axis=0)
        label_overrides = {
            "NESTEDSIN": (-10, -18, "right", "top"),
            "SIN": (-10, 15, "right", "bottom"),
            "SINFOURIER": (10, -15, "left", "top"),
            "experimental_sinusoidalYaw": (10, -15, "left", "top"),
            "yawPower": (-10, 12, "right", "bottom"),
        }
        for index, (record, point) in enumerate(zip(records, coordinates)):
            place_right = point[0] >= center[0]
            vertical = 8 if index % 2 == 0 else -13
            offset = label_overrides.get(record["gait"])
            if offset:
                horizontal, vertical, horizontal_alignment, vertical_alignment = offset
            else:
                horizontal = 8 if place_right else -8
                horizontal_alignment = "left" if place_right else "right"
                vertical_alignment = "bottom" if vertical > 0 else "top"
            axis.annotate(
                compact_gait_name(record["gait"]), point,
                xytext=(horizontal, vertical),
                textcoords="offset points",
                ha=horizontal_alignment,
                va=vertical_alignment,
                fontsize=10, fontweight="semibold", color="#121F35",
                zorder=4,
            )

    axis.axhline(0, color="#CBD2DB", linewidth=0.8, zorder=0)
    axis.axvline(0, color="#CBD2DB", linewidth=0.8, zorder=0)
    axis.set_xlabel(
        f"Similarity direction 1 ({explained[0] * 100:.0f}% of the differences)",
        fontsize=14, fontweight="bold",
    )
    axis.set_ylabel(
        f"Similarity direction 2 ({explained[1] * 100:.0f}% of the differences)",
        fontsize=14, fontweight="bold",
    )
    axis.tick_params(labelsize=11)
    axis.grid(True, color="#E1E5EA", linewidth=0.7)
    axis.spines[["top", "right"]].set_visible(False)
    axis.legend(
        loc="upper left", frameon=False, fontsize=11, ncol=max(1, cluster_count),
        title="Similar gait groups", title_fontsize=11,
    )
    add_similarity_equation(fig, left=0.07)
    fig.text(
        0.07, 0.025,
        "Pg, P2g, and P3g are the PSD at the gait frequency and its second and third harmonics. Larger dots include more flipper designs.",
        fontsize=9.5, color="#68707D",
    )
    fig.subplots_adjust(left=0.11, right=0.97, top=0.84, bottom=0.23)
    fig.savefig(output_path, facecolor="white")
    plt.close(fig)
    return output_path


def save_force_tradeoff(spectral_rows, output_path):
    """Contrast average absolute Fy with variation across repeated runs."""
    palette = {
        "control": "#6B7280",
        "fiberglass": "#7C3AED",
        "petg_thin": "#F59E0B",
        "rib_flipper": "#16A34A",
        "stripe_flipper": "#2563EB",
    }
    fig, axis = plt.subplots(figsize=(16, 9), dpi=160)
    fig.suptitle(
        "Average Fy and run-to-run variation",
        x=0.07, y=0.97, ha="left", fontsize=24,
        fontweight="bold", color="#121F35",
    )
    fig.text(
        0.07, 0.915,
        "Each dot summarizes one flipper and gait. Higher means its average "
        "|Fy| changed more between repeated runs.",
        fontsize=12.2, color="#5C6572",
    )
    draw_force_variation_panel(
        axis, spectral_rows, "Fy", palette, show_title=False
    )
    handles, labels = axis.get_legend_handles_labels()
    if handles:
        axis.legend(
            handles, labels, loc="upper left", frameon=False,
            fontsize=11, ncol=2,
        )
    axis.set_xlabel(
        "Average |Fy| across runs (N)", fontsize=15, fontweight="bold"
    )
    axis.set_ylabel(
        "Run-to-run SD of average |Fy| (N)",
        fontsize=15, fontweight="bold",
    )
    axis.tick_params(labelsize=12)
    fig.text(
        0.07, 0.055,
        "Only combinations with at least two valid runs are shown. "
        "Larger dots mean more runs.",
        fontsize=10.5, color="#68707D",
    )
    fig.subplots_adjust(left=0.10, right=0.97, top=0.84, bottom=0.14)
    fig.savefig(output_path, facecolor="white")
    plt.close(fig)
    return output_path


def draw_force_variation_panel(
        axis, spectral_rows, force_axis, palette, show_title=True):
    """Draw mean run-average force versus its variation between runs."""
    records = [
        record for record in aggregate_force_spectral_rows(
            spectral_rows, force_axis
        )
        if np.isfinite(record.get("mean_abs_n", np.nan))
        and np.isfinite(record.get("std_n", np.nan))
        and record.get("runs", 0) >= 2
    ]
    for flipper in FLIPPERS:
        subset = [record for record in records if record["flipper"] == flipper]
        if not subset:
            continue
        sizes = [70 + 28 * min(record["runs"], 10) for record in subset]
        axis.scatter(
            [record["mean_abs_n"] for record in subset],
            [record["std_n"] for record in subset],
            s=sizes, color=palette[flipper], alpha=0.80,
            edgecolor="white", linewidth=1.1, label=friendly(flipper),
        )

    notable = []
    if records:
        for candidate in (
            max(records, key=lambda record: record["mean_abs_n"]),
            max(records, key=lambda record: record["std_n"]),
        ):
            if candidate not in notable:
                notable.append(candidate)
    x_median = np.median([record["mean_abs_n"] for record in records]) if records else 0
    for record in notable:
        right_side = record["mean_abs_n"] > x_median
        axis.annotate(
            f"{friendly(record['flipper'])}\n{compact_gait_name(record['gait'])}",
            (record["mean_abs_n"], record["std_n"]),
            xytext=(-7 if right_side else 7, 7),
            textcoords="offset points",
            ha="right" if right_side else "left", va="bottom",
            fontsize=8.5, color="#121F35",
        )

    if show_title:
        axis.set_title(force_axis, fontsize=17, fontweight="bold")
    average_label = (
        "Average resultant magnitude across runs (N)"
        if force_axis.lower() == "net"
        else f"Average |{force_axis}| across runs (N)"
    )
    variation_label = (
        "Run-to-run SD of average resultant magnitude (N)"
        if force_axis.lower() == "net"
        else f"Run-to-run SD of average |{force_axis}| (N)"
    )
    axis.set_xlabel(
        average_label, fontsize=12.5, fontweight="bold"
    )
    axis.set_ylabel(
        variation_label, fontsize=12.5, fontweight="bold"
    )
    axis.tick_params(labelsize=10)
    axis.grid(True, color="#D9DDE3", linewidth=0.7)
    axis.spines[["top", "right"]].set_visible(False)
    if records:
        axis.set_xlim(left=0)
        axis.set_ylim(bottom=0)


def save_other_axis_force_tradeoffs(spectral_rows, output_path):
    """Show how Fx and Fz run averages vary across repeated gait runs."""
    palette = {
        "control": "#6B7280",
        "fiberglass": "#7C3AED",
        "petg_thin": "#F59E0B",
        "rib_flipper": "#16A34A",
        "stripe_flipper": "#2563EB",
    }
    fig, axes = plt.subplots(1, 2, figsize=(16, 9), dpi=160)
    fig.suptitle(
        "Average Fx and Fz across repeated runs",
        x=0.065, y=0.97, ha="left", fontsize=24,
        fontweight="bold", color="#121F35",
    )
    fig.text(
        0.065, 0.915,
        "Each dot summarizes one flipper and gait. Higher means its average "
        "force changed more from one run to another.",
        fontsize=12.2, color="#5C6572",
    )
    for axis, force_axis in zip(axes, ("Fx", "Fz")):
        draw_force_variation_panel(axis, spectral_rows, force_axis, palette)
    handles, labels = axes[0].get_legend_handles_labels()
    if handles:
        fig.legend(
            handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.87),
            frameon=False, fontsize=10.5, ncol=len(labels),
        )
    fig.text(
        0.065, 0.055,
        "Only combinations with at least two valid runs are shown. "
        "Larger dots mean more runs.",
        fontsize=10.5, color="#68707D",
    )
    fig.subplots_adjust(
        left=0.08, right=0.98, top=0.78, bottom=0.14, wspace=0.24
    )
    fig.savefig(output_path, facecolor="white")
    plt.close(fig)
    return output_path


def save_net_force_tradeoff(spectral_rows, output_path):
    """Show how average resultant force varies across repeated gait runs."""
    palette = {
        "control": "#6B7280",
        "fiberglass": "#7C3AED",
        "petg_thin": "#F59E0B",
        "rib_flipper": "#16A34A",
        "stripe_flipper": "#2563EB",
    }
    fig, axis = plt.subplots(figsize=(16, 9), dpi=160)
    fig.suptitle(
        "Average resultant magnitude and run-to-run variation",
        x=0.07, y=0.97, ha="left", fontsize=24,
        fontweight="bold", color="#121F35",
    )
    fig.text(
        0.07, 0.915,
        "Each dot summarizes one flipper and gait. Higher means its average "
        "resultant magnitude changed more from one run to another.",
        fontsize=12.2, color="#5C6572",
    )
    draw_force_variation_panel(
        axis, spectral_rows, "Net", palette, show_title=False
    )
    handles, labels = axis.get_legend_handles_labels()
    if handles:
        axis.legend(
            handles, labels, loc="upper left", frameon=False,
            fontsize=11, ncol=2,
        )
    axis.set_xlabel(
        "Average resultant magnitude across runs (N)", fontsize=15, fontweight="bold"
    )
    axis.set_ylabel(
        "Run-to-run SD of average resultant magnitude (N)",
        fontsize=15, fontweight="bold",
    )
    axis.tick_params(labelsize=12)
    fig.text(
        0.07, 0.055,
        "Only combinations with at least two valid runs are shown. "
        "Larger dots mean more runs.",
        fontsize=10.5, color="#68707D",
    )
    fig.subplots_adjust(left=0.11, right=0.97, top=0.84, bottom=0.14)
    fig.savefig(output_path, facecolor="white")
    plt.close(fig)
    return output_path


def discover_groups(bag_dir):
    groups = defaultdict(lambda: defaultdict(list))
    for path in bag_dir.glob("*.bag"):
        flipper = identify_flipper(path.name)
        if flipper is None:
            continue
        gait, run = parse_gait_and_run(path, flipper)
        groups[flipper][gait].append((run, path))

    for gait_map in groups.values():
        for entries in gait_map.values():
            entries.sort(key=lambda item: (item[0], natural_key(item[1].name)))
    return groups


def save_signed_run_comparison(rows, output_path):
    rows=valid_spectral_rows(rows)
    fig,axs=plt.subplots(3,1,figsize=(16,9),sharex=True)
    variants=sorted({r['gait'] for r in rows},key=gait_sort_key)
    fig.suptitle('Signed force by parameter run',x=.065,y=.97,ha='left',fontsize=25,fontweight='bold')
    fig.text(.065,.92,'Each point is one recorded run. Positive and negative retain load-cell axis direction. Lines connect run order only.',fontsize=12)
    for axis,prefix in zip(axs,['fx','fy','fz']):
        for flipper in FLIPPERS:
            rr=[r for r in rows if r['flipper']==flipper]
            if not rr:continue
            xx=[variants.index(r['gait']) for r in rr]
            yy=[r.get(prefix+'_mean_n',np.nan) for r in rr]
            order=np.argsort(xx)
            axis.plot(np.array(xx)[order],np.array(yy)[order],'o-',lw=1,label=friendly(flipper))
        axis.axhline(0,color='black',lw=.7);axis.grid(alpha=.2)
        axis.set_ylabel(prefix.capitalize()+' mean (N)')
    axs[0].legend(ncol=5,fontsize=9)
    axs[-1].set_xticks(range(len(variants)),[compact_gait_name(v) for v in variants],fontsize=9)
    fig.text(.065,.04,'Mean force = integral F(t) dt / active duration for uniformly sampled data (sample mean used). This compares settings, not replicate uncertainty.',fontsize=10)
    fig.subplots_adjust(left=.09,right=.97,top=.86,bottom=.14,hspace=.15)
    fig.savefig(output_path,dpi=150);plt.close(fig);return output_path


def save_global_feature_similarity(rows,output_path,force_axis='Fy'):
    # Every run number is distinct. Medians across materials are explicitly disclosed.
    records=aggregate_gait_spectral_rows(rows,force_axis)
    cols=['gait_frequency_hz','second_harmonic_ratio','third_harmonic_ratio','spectral_entropy']
    records=[r for r in records if all(np.isfinite(r.get(c,np.nan)) for c in cols)]
    if len(records)<2:return None
    x=np.array([[np.log10(max(r[cols[0]],1e-9)),np.log10(1+max(r[cols[1]],0)),np.log10(1+max(r[cols[2]],0)),r[cols[3]]] for r in records])
    scale=np.percentile(x,75,axis=0)-np.percentile(x,25,axis=0)
    scale=np.where(scale>1e-12,scale,np.std(x,axis=0));scale=np.where(scale>1e-12,scale,1)
    z=(x-np.median(x,axis=0))/scale
    dist=np.linalg.norm(z[:,None,:]-z[None,:,:],axis=2)
    sim=1/(1+dist)
    fig,ax=plt.subplots(figsize=(16,9))
    im=ax.imshow(sim,vmin=0,vmax=1,cmap='viridis')
    labels=[f"{GAIT_ORDER.index(r['gait'].split('::')[0])+1}.{r['gait'].split('::')[1]}" if r['gait'].split('::')[0] in GAIT_ORDER else r['gait'] for r in records]
    ax.set_xticks(range(len(labels)),labels,rotation=90,fontsize=8)
    ax.set_yticks(range(len(labels)),labels,fontsize=8)
    fig.colorbar(im,ax=ax,fraction=.025,pad=.02,label='Feature similarity 1/(1 + distance)')
    fig.suptitle(force_axis+' spectral-feature similarity across parameter runs',x=.05,y=.975,ha='left',fontsize=23,fontweight='bold')
    fig.text(.05,.925,'Each ID is family.run. Features use medians across available flippers for that run. Material coverage may differ.',fontsize=11)
    key='\n'.join(f'{i+1}. {GAIT_DISPLAY_NAMES[g]}' for i,g in enumerate(GAIT_ORDER))
    fig.text(.71,.80,key,fontsize=10,va='top',linespacing=1.7)
    fig.text(.05,.045,'Features: log10(f), log10(1+P(2f)/P(f)), log10(1+P(3f)/P(f)), spectral entropy.\nNormalize by median and IQR (SD fallback). Distance = Euclidean distance. This compares selected features, not full spectra.',fontsize=10)
    fig.subplots_adjust(left=.07,right=.68,top=.85,bottom=.13)
    fig.savefig(output_path,dpi=150);plt.close(fig);return output_path


def main():
    global PARAMETER_SOURCE
    # Local imports permit parameter-only plotting without a ROS installation.
    if '--parameters-only' in sys.argv:
        parser = argparse.ArgumentParser()
        parser.add_argument('--parameters-only', action='store_true')
        parser.add_argument('--parameter-source', choices=['pdf','code'], default='pdf')
        parser.add_argument('--report-dir', type=Path, default=Path('gait_reanalysis'))
        opts = parser.parse_args()
        PARAMETER_SOURCE = opts.parameter_source
        export_parameter_report(opts.report_dir)
        return
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bag-dir",
        type=Path,
        default=Path("/home/odinroast/crab_ws/bags"),
        help="Directory containing .bag MCAP files",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("flipper_all_run_graphs_smoothed.pptx"),
        help="Output PowerPoint path",
    )
    parser.add_argument(
        "--keep-plots",
        type=Path,
        help="Optional directory in which generated PNG graphs are retained",
    )
    parser.add_argument(
        "--hampel-seconds",
        type=float,
        default=0.10,
        help="Local Hampel outlier window in seconds (default: 0.10)",
    )
    parser.add_argument(
        "--smooth-seconds",
        type=float,
        default=0.25,
        help="Rolling-median smoothing window in seconds (default: 0.25)",
    )
    parser.add_argument(
        "--hampel-sigma",
        type=float,
        default=3.5,
        help="Hampel rejection threshold in robust sigma (default: 3.5)",
    )
    parser.add_argument(
        "--independent-axes",
        action="store_true",
        help=(
            "Use Matplotlib autoscaling for each run instead of common "
            "force and servo Y-axis limits across the entire deck"
        ),
    )
    parser.add_argument(
        "--diagnostics-output",
        type=Path,
        default=Path("force_scale_diagnostics.csv"),
        help=(
            "CSV comparing raw and post-filter force extrema "
            "(default: force_scale_diagnostics.csv)"
        ),
    )
    parser.add_argument(
        "--force-scale-percentile",
        type=float,
        default=99.9,
        help=(
            "Pooled post-filter absolute-force percentile used for the "
            "shared Y-axis (default: 99.9)"
        ),
    )
    parser.add_argument(
        "--load-sample-rate",
        type=float,
        default=10_000.0,
        help="Acquisition rate of individual load-cell rows in Hz (default: 10000)",
    )
    parser.add_argument(
        "--spectral-max-frequency",
        type=float,
        default=50.0,
        help="Highest frequency included in spectral features (default: 50 Hz)",
    )
    parser.add_argument(
        "--spectral-output",
        type=Path,
        default=Path("spectral_features.csv"),
        help="Per-run time/frequency/response feature CSV",
    )
    parser.add_argument('--parameter-source', choices=['pdf','code'], default='pdf',
                        help='PDF table values, or documented controller alternatives for conflicts')
    parser.add_argument('--report-dir', type=Path, default=Path('gait_reanalysis'))
    args = parser.parse_args()
    PARAMETER_SOURCE = args.parameter_source
    if rosbag2_py is None:
        raise SystemExit('ROS imports unavailable. Source your ROS and crab_ws environments, or use --parameters-only.')

    if not args.bag_dir.is_dir():
        raise SystemExit(f"Bag directory does not exist: {args.bag_dir}")
    if args.hampel_seconds <= 0 or args.smooth_seconds <= 0:
        raise SystemExit("Hampel and smoothing windows must be greater than zero")
    if args.hampel_sigma <= 0:
        raise SystemExit("Hampel sigma must be greater than zero")
    if not 90.0 <= args.force_scale_percentile <= 100.0:
        raise SystemExit("Force scale percentile must be between 90 and 100")
    if args.load_sample_rate <= 0 or args.spectral_max_frequency <= 0:
        raise SystemExit("Sample rate and spectral maximum frequency must be positive")
    if args.spectral_max_frequency >= args.load_sample_rate / 2:
        raise SystemExit("Spectral maximum frequency must be below Nyquist")

    groups = discover_groups(args.bag_dir)
    total_bags = sum(
        len(entries)
        for gait_map in groups.values()
        for entries in gait_map.values()
    )
    gait_count = sum(len(gait_map) for gait_map in groups.values())
    if total_bags == 0:
        raise SystemExit(f"No recognized .bag files found in {args.bag_dir}")

    print("Scanning cleaned data for summary metrics and plot limits...")
    scanned_axes, scaling_errors, force_diagnostics, spectral_features = scan_global_axes(
        groups,
        args.hampel_seconds,
        args.smooth_seconds,
        args.hampel_sigma,
        args.force_scale_percentile,
        args.load_sample_rate,
        args.spectral_max_frequency,
    )
    global_axes = None if args.independent_axes else scanned_axes
    if not args.independent_axes:
        print(
            "Common axes: "
            f"force={global_axes['force']}, "
            f"servo={global_axes['servo']}"
        )
        print(
            "Full observed post-filter force range: "
            f"{global_axes['force_observed']}"
        )
    write_force_diagnostics(args.diagnostics_output, force_diagnostics)
    print_suspicious_force_peaks(force_diagnostics)
    print(f"\nSaved diagnostics: {args.diagnostics_output.resolve()}")
    write_dict_rows(args.spectral_output, spectral_features)
    successful_spectra = sum(r.get("status") == "ok" for r in spectral_features)
    print(
        f"Saved spectral features: {args.spectral_output.resolve()} "
        f"({successful_spectra}/{len(spectral_features)} runs analyzed)"
    )

    # Every run retains a separate identity in comparisons; settings are never pooled by family.
    export_parameter_report(args.report_dir)
    plot_dir = args.keep_plots or args.report_dir / 'measured_plots'
    plot_dir.mkdir(parents=True, exist_ok=True)
    mapping=[]
    for flipper,gait_map in groups.items():
        for gait,entries in gait_map.items():
            for run,bag in entries:
                p=parameters_for(gait,run)
                mapping.append(dict(bag=bag.name,flipper=flipper,gait=gait,run=run,
                    expected_frequency_hz=p['frequency_hz'] if p else '',
                    parameters=parameter_summary(p), notes=p['notes'] if p else 'UNMAPPED RUN',
                    mapping_basis='Assumed table row equals filename run number'))
    write_dict_rows(args.report_dir / 'bag_parameter_mapping.csv',mapping)
    prs = Presentation()
    prs.slide_width = SLIDE_W
    prs.slide_height = SLIDE_H
    add_section_slide(prs, 'Gait analysis by parameter run', 'Recorded evidence and motion models',
        'Sin-Sin first, then Sin-Square variations, then Fourier. Each table row stays separate.')
    add_section_slide(prs, 'How to read this analysis', 'Sources and assumptions',
        'Model plots use PDF table values by default. Table row N is assumed to match filename run N. '
        'Dashed model curves show conflicting controller values. Recorded commands and forces remain measured data. '
        'Servo IDs are authoritative. Encoder degrees = counts × 360/4096, referenced to encoder zero. Resultant magnitude has no direction; signed Fx/Fy/Fz retain direction.')
    errors=list(scaling_errors)
    skipped={name for name,_ in scaling_errors}
    for gait in GAIT_ORDER + tuple(sorted({g for gm in groups.values() for g in gm} - set(GAIT_ORDER))):
        if not any(gait in gm for gm in groups.values()): continue
        add_section_slide(prs, friendly(gait), 'Gait family',
                          f"Original filename key: {gait}. Rows represent distinct parameter runs.")
        table=args.report_dir/'parameter_plots'/f'{GAIT_ORDER.index(gait)+1:02d}_{gait}_table.png' if gait in GAIT_ORDER else None
        if table and table.exists(): add_graph_slide(prs,table)
        for p in RUN_PARAMETERS.get(gait,[]):
            model=args.report_dir/'parameter_plots'/f"{GAIT_ORDER.index(gait)+1:02d}_{gait}_run_{p['run']:02d}.png"
            add_graph_slide(prs,model)
        variant_groups=defaultdict(lambda:defaultdict(list))
        for flipper in FLIPPERS:
            entries=groups.get(flipper,{}).get(gait,[])
            if not entries:continue
            add_section_slide(prs,friendly(flipper),'Measured runs',friendly(gait))
            for run,bag_path in entries:
                variant_groups[flipper][f'{gait}::{run}'].append((run,bag_path))
                if bag_path.name in skipped:continue
                try:
                    print(f'Reading {bag_path.name}')
                    data=read_bag(bag_path)
                    graph=save_run_graph(data,plot_dir/f'{flipper}__{gait}__run_{run}',flipper,gait,run,
                        hampel_seconds=args.hampel_seconds,smooth_seconds=args.smooth_seconds,
                        hampel_sigma=args.hampel_sigma,global_axes=global_axes)
                    add_graph_slide(prs,graph)
                except Exception as exc:
                    errors.append((bag_path.name,str(exc)))
        dr=[dict(r,gait=f"{gait}::{r['run']}") for r in force_diagnostics if r['gait']==gait]
        sr=[dict(r,gait=f"{gait}::{r['run']}") for r in spectral_features if r['gait']==gait]
        if dr:
            add_graph_slide(prs,save_sustained_force_summary(variant_groups,dr,plot_dir/f'{gait}_force_by_run.png'))
        if valid_spectral_rows(sr):
            add_graph_slide(prs,save_spectral_summary(variant_groups,sr,plot_dir/f'{gait}_spectra_by_run.png'))
            add_graph_slide(prs,save_combined_force_frequency_summary(dr,sr,plot_dir/f'{gait}_frequency_force_by_run.png'))
            add_graph_slide(prs,save_signed_run_comparison(sr,plot_dir/f'{gait}_signed_force_variation.png'))
            if len(aggregate_gait_spectral_rows(sr)) >= 3:
                add_graph_slide(prs,save_other_axis_similarity_maps(sr,plot_dir/f'{gait}_similarity.png'))
    all_variants=[dict(r,gait=f"{r['gait']}::{r['run']}") for r in spectral_features]
    if valid_spectral_rows(all_variants):
        add_section_slide(prs,'Cross-gait comparisons','Spectral features','Every parameter row remains a separate item. Features are aggregated across available flippers.')
        for force_axis in ('Fx','Fy','Fz','Net'):
            path=save_global_feature_similarity(all_variants,plot_dir/f'all_gaits_similarity_{force_axis}.png',force_axis)
            if path: add_graph_slide(prs,path)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    prs.save(args.output)
    if errors: write_dict_rows(args.report_dir/'errors.csv',[{'bag':n,'error':e} for n,e in errors])
    print(f'Saved {args.output.resolve()} ({len(prs.slides)} slides). Errors: {len(errors)}')


if __name__ == '__main__':
    main()