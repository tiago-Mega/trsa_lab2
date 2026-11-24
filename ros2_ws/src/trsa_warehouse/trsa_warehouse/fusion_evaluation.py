"""Time-aligned SE(2) drift comparison. Evaluation only; no estimator feedback."""
from bisect import bisect_left
import math
from .fusion_math import relative_pose, wrap


def interpolate(samples,t,max_gap=0.2):
    times=[s[0] for s in samples]
    return interpolate_indexed(samples,times,t,max_gap)


def interpolate_indexed(samples,times,t,max_gap=0.2):
    i=bisect_left(times,t)
    if i<len(samples) and abs(times[i]-t)<1.e-9: return samples[i][1:4]
    if i==0 or i==len(samples): return None
    a,b=samples[i-1],samples[i]
    gap=b[0]-a[0]
    if gap<=0 or gap>max_gap: return None
    u=(t-a[0])/gap
    return (a[1]+u*(b[1]-a[1]),a[2]+u*(b[2]-a[2]),wrap(a[3]+u*wrap(b[3]-a[3])))


def compare(streams,max_gap=0.2):
    cleaned={}
    for key in ('wheel','ekf','truth'):
        # Sorting and duplicate removal are for evaluation only.
        unique={s[0]:tuple(s[:4]) for s in streams.get(key,[]) if len(s)>=4 and all(math.isfinite(v) for v in s[:4])}
        cleaned[key]=[unique[t] for t in sorted(unique)]
        if len(cleaned[key])<2: raise ValueError('Insufficient samples for '+key)
    times={k:[s[0] for s in v] for k,v in cleaned.items()}
    rows=[]; origins=None
    for truth in cleaned['truth']:
        t=truth[0]
        poses={k:interpolate_indexed(cleaned[k],times[k],t,max_gap) for k in ('wheel','ekf')}
        if any(v is None for v in poses.values()): continue
        poses['truth']=truth[1:4]
        if origins is None: origins=poses.copy()
        rel={k:relative_pose(v,origins[k]) for k,v in poses.items()}
        row={'sim_time':t}
        for key in ('wheel','ekf'):
            row[key+'_position_error_m']=math.hypot(rel[key][0]-rel['truth'][0],rel[key][1]-rel['truth'][1])
            row[key+'_yaw_error_rad']=wrap(rel[key][2]-rel['truth'][2])
        rows.append(row)
    if len(rows)<2: raise ValueError('No common timestamp interval with sufficiently fresh bracketing samples')
    summary={'paired_samples':len(rows),'sim_duration_s':rows[-1]['sim_time']-rows[0]['sim_time'],
             'alignment':'Each trajectory aligned at the same first common timestamp; SE(2) drift, not global ATE',
             'interpolation_max_gap_s':max_gap,'origins':origins}
    for key in ('wheel','ekf'):
        pos=[r[key+'_position_error_m'] for r in rows]; angle=[r[key+'_yaw_error_rad'] for r in rows]
        summary[key]={'position_rmse_m':math.sqrt(sum(x*x for x in pos)/len(pos)),
                      'yaw_rmse_rad':math.sqrt(sum(x*x for x in angle)/len(angle)),
                      'final_position_error_m':pos[-1],'final_yaw_error_rad':angle[-1]}
    return rows,summary
