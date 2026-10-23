"""Rebuild a nine-page report from separately labelled offline and Webots evidence."""
from pathlib import Path
import json, os
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import Paragraph
from reportlab.lib.styles import ParagraphStyle
from PIL import Image
from pypdf import PdfReader, PdfWriter
P=Path(__file__).resolve().parent
R=json.loads((P/'results/tables/metrics.json').read_text())
M=json.loads((P/'results/webots/measured_results.json').read_text())
C=json.loads((P/'results/webots/connected/measured_results.json').read_text())
fonts=Path(os.environ.get('WINDIR','C:/Windows'))/'Fonts'
for n,f in [('Times','times.ttf'),('Times-Bold','timesbd.ttf'),('Times-Italic','timesi.ttf')]:pdfmetrics.registerFont(TTFont(n,str(fonts/f)))
pdfmetrics.registerFontFamily('Times',normal='Times',bold='Times-Bold',italic='Times-Italic',boldItalic='Times-Bold')
style=ParagraphStyle('body',fontName='Times',fontSize=12,leading=18)
W,H=A4;out=P/'report/body.pdf';c=canvas.Canvas(str(out),pagesize=A4);page=1

def start(title):
 global y,page
 page+=1;y=H-48;c.setFont('Times-Bold',12);c.drawString(45,y,title);y-=30
 c.setFont('Times',12);c.drawRightString(W-45,24,str(page))
def para(s):
 global y
 p=Paragraph(s,style);_,h=p.wrap(W-90,1000)
 if y-h<46:raise ValueError(f'Page {page} overflow: {s[:60]}')
 p.drawOn(c,45,y-h);y-=h+10

def fig(path,height,caption):
 global y
 im=Image.open(P/path);width=min(W-90,height*im.width/im.height);height=width*im.height/im.width
 if y-height<46:raise ValueError(f'Image overflow {page}')
 c.drawImage(str(P/path),(W-width)/2,y-height,width,height);y-=height+8;para(caption)
def end():c.showPage()

start('Contents and introduction')
para('Contents: introduction (2); Component A methods (3); offline A results (4); measured Webots A results (5); multi-agent methods and live results (6); offline learning and swarm results (7); ethics and limitations (8); conclusion and references (9).')
para('The project investigates perception, graph search and differential-drive control for one robot, followed by coordination among four robots. The assignment requires Webots simulations, reproducible Python code and Matplotlib analysis (Richfield Graduate Institute of Technology, n.d.).')
para('Two evidence sources are kept separate. Offline reference experiments use seeded synthetic sensing, idealised dynamics, PSO and a neural classifier. Live trials use Webots R2025a on Windows 11 with Python 3.14.0, an Intel i5-11320H and an NVIDIA RTX 3050 Laptop GPU. All reported simulator outcomes come from saved controller or supervisor logs. No physical robot was tested.')
para('The repaired worlds use custom differential-drive robots, GPS/IMU simulation pose, native obstacles and a procedural parquet checkerboard floor. The wooden appearance is visual geometry; the original collision floor remains unchanged. Worlds start at their defined initial positions. A separate observer measures pose, uprightness and above-floor contacts every 32 ms without commanding the robots.')
para('Reproducibility: run verify_setup.py and scripts/run_all.py, then open the repaired worlds in Webots. The observer pauses A after 180.064 s and B after 120.064 s. Raw CSVs, sensor exports, screenshots and analysis plots accompany this report. The README explains dependencies, run order and evidence limitations.')
end()
start('Component A methods: A1-A4')
para('<b>A1 - sensing.</b> A 180-beam LiDAR updates a 0.1 m log-odds occupancy grid. Camera processing uses blur, red HSV segmentation, morphology and contour filtering. Detection bearings are paired with nearby LiDAR ranges to fuse semantic landmarks with metric depth. GPS/IMU provide ground-truth pose; this is not autonomous localisation. Real image and scan exports are retained separately from synthetic examples.')
para('<b>A2 - planning.</b> Dijkstra and A* share an eight-connected grid and prohibit diagonal corner cutting. Euclidean distance is an admissible A* heuristic. The offline comparison uses the same map and 0.18 m inflation; live navigation uses 0.25 m inflation and replans as its sensed map changes. Equal optimal graph cost does not establish optimal continuous robot motion.')
para('<b>A3 - control.</b> Differential-drive kinematics use dx/dt = v cos(theta), dy/dt = v sin(theta), dtheta/dt = omega. Wheel rates are (v - omega L/2)/r and (v + omega L/2)/r, with r = 0.033 m and L = 0.160 m. Live heading PID gains are Kp=5, Ki=0.05, Kd=0.22, with anti-windup and a 2 rad/s angular limit. Forward speed is 0.10 m/s only when heading error is below 0.35 rad.')
para('<b>A4 - integration.</b> Sensing and planning run every fifth 32 ms step. The controller follows successive waypoints, clears an invalid route and stops within 0.16 m of (5.45, 5.45). The independent observer counts contacts above z=0.012 m, excluding the floor. This diagnostic does not establish real-world safety.')
para('Tracking error is the shortest distance from each measured position to the polyline of its most recent nonempty saved plan. RMS is calculated across available samples. Plans change during mapping, so this metric is not error against one globally optimal reference path.')
end()
start('Component A: offline reference results')
a=R['A2'];fig('results/figures/A2_planning.png',205,'Dijkstra and A* on the identical offline occupancy graph.')
para(f"Both searches returned {a['astar']['length_m']:.3f} m. Dijkstra expanded {a['dijkstra']['nodes_expanded']} nodes versus {a['astar']['nodes_expanded']} for A*. Median times over seven repeats were {a['dijkstra']['runtime_ms_median']} and {a['astar']['runtime_ms_median']} ms respectively on this computer.")
fig('results/figures/A3_pid.png',190,'Offline heading-step responses with assumed actuator lag and disturbance.')
p=R['A3']['PID'];para(f"The illustrated PID (10, 0.6, 2) gave {p['overshoot_pct']}% overshoot, {p['rise_s']} s rise time, {p['settle_s']} s settling time and {p['steady_state_error_deg']} degrees final error. These gains differ from the live navigation gains. The idealised response is not a measured Webots motor response or a general stability proof.")
end()
start('Component A: measured Webots retest')
a=M['A'];fig('results/webots/A_end.png',235,'Actual Webots viewport at the end of the parquet-floor navigation trial.')
para(f"The robot ran for {a['duration_s']:.1f} s. Goal reached: {a['goal_reached']}; first accepted at {a['first_goal_time_s']} s. Travel was {a['travel_distance_m']:.3f} m, final goal error {a['final_goal_distance_m']:.3f} m, with {a['replans']} replans. RMS distance to the active plan was {a.get('rms_distance_to_active_plan_m',0):.3f} m.")
fig('results/webots/actual_webots_trajectories.png',190,'Measured trajectories; dashed lines show the saved navigation plans.')
para(f"Above-floor contact rows: {M['A_physics']['sample_rows_with_contact_above_floor']}. Landmark detections appeared in {a['landmark_detection_rows']} logged rows; rows are not unique landmarks. Raw frames, LiDAR scans and the final log-odds map allow inspection of perception errors.")
end()
start('Component B: team design and live retests')
para('Four robots perform a cooperative coverage-station task. Each uses local A* routing and eight proximity sensors. Stations are computed by offline ring-topology PSO, then projected to free cells with 0.20 m clearance inflation. Empty routes cause a stop. A station is accepted within 0.08 m of its safe projected location; this differs from the unconstrained PSO coordinate.')
para('Each agent broadcasts its estimate and pose and applies a consensus update with weight 0.25 per received peer. The 1 m radio world can partition. A separately labelled 4 m range diagnostic provides full-arena connectivity. It is not evidence that the original 1 m configuration converges, and station optimisation is not running as distributed PSO inside Webots.')
fig('results/webots/B_end.png',210,'Actual four-robot Webots world, after the 1 m radio test.')
b=M['B'];para(f"At 120 s, {sum(v.get('safe_goal_reached',False) for v in b.values())}/4 agents met the safe-station tolerance. Final estimate spread was {M['B_final_consensus_spread']:.6f}; above-floor contact rows: {M['B_physics']['sample_rows_with_contact_above_floor']}. In the 4 m diagnostic, spread was {C['B_final_consensus_spread']:.6g} and contact rows were {C['B_physics']['sample_rows_with_contact_above_floor']}.")
para('The neural action is used in proximity avoidance, with a hard threshold override and route-based motion elsewhere. Successful route following alone therefore does not prove the neural policy is responsible or robust. Live centralised-versus-distributed trials and fault-injection experiments remain incomplete.')
end()
start('Component B: offline swarm and learning evidence')
fig('results/figures/B3_training.png',205,'Synthetic holdout confusion matrix and neural training loss.')
b=R['B3'];para(f"The MLP uses {b['synthetic_train_samples']} synthetic training samples and {b['synthetic_test_samples']} held-out samples. Accuracy was {100*b['synthetic_test_accuracy']:.2f}%. Labels imitate a proximity expert. This is a synthetic classification score, not a Webots navigation success rate, and the model has not been retrained on the saved live sensor data.")
fig('results/figures/B2_pso.png',190,'Offline PSO convergence and coverage-station positions.')
p=R['B2']['pso'];para(f"Final coverage costs were {p['global']['final_cost']} (global topology), {p['ring']['final_cost']} (ring) and {p['random_baseline_cost']} (random baseline). Lower is better under this objective. The offline connected consensus spread was {R['B2']['final_consensus_spread']:.6g}.")
para('The centralised/distributed comparison in B3_coordination.png is an offline communication-and-coverage proxy, not a matched Webots experiment. Its modelling assumptions must accompany any comparison; it cannot establish a deployed multi-agent advantage.')
end()
start('B4: ethics, reliability and responsible autonomy')
para('<b>Bias and domain shift.</b> Synthetic proximity vectors, fixed red landmarks and an idealised LiDAR model underrepresent real hazards. Shadows, reflective objects, dust and changing floor colours can affect perception. Evaluate held-out arenas and sensor noise separately, and prioritise false negatives near people over harmless detours. Saved live frames make errors inspectable rather than hiding them behind one accuracy score.')
para('<b>Explainability.</b> A neural prediction is not a safety explanation. Log the sensor inputs, selected action, override, route and model version. Explain which part of the controller actually caused movement. Preserve failures and configuration changes; do not relabel a connectivity-control experiment as success for the original network.')
para('<b>Safety and human responsibility.</b> Simulation lacks many physical uncertainties. Speed limits, stop behaviour and clearance inflation reduce specific risks but are not certified safeguards. A real deployment needs emergency stopping, supervised commissioning, hazard analysis and accountability for incidents. A contact-count diagnostic is narrower than collision-free operation in all conditions.')
para('<b>Fault tolerance.</b> The 1 m topology can isolate agents. Stale consensus values should not be treated as team agreement. Future tests should cover dropped packets, stopped robots, sensor loss and blocked routes, measuring task completion and separation after failures. The current experiments do not validate those cases.')
para('<b>South African deployment context.</b> Power interruptions, uneven connectivity, dust and limited maintenance resources can change reliability. Use accessible local-language operator instructions and consult affected workers about monitoring and role changes. Where video or location data identify people, obtain institutional guidance on lawful collection, access and retention; minimise personal data and protect incident records.')
para('<b>Academic integrity.</b> Separate measured results from synthetic demonstrations, acknowledge assistance as required by the institution, and review the code personally. The supplied declaration must be completed and signed by the student only when its statements are accurate.')
end()
start('Conclusion and references')
para('The project now contains reproducible offline reference experiments and instrumented Webots trials. Repairs address robot geometry, sensor ordering, model compatibility and unsafe no-path movement. The parquet appearance is included in the tested worlds, and the generator restores initial positions for repeat runs.')
para('A* reduced explored nodes without changing the tested graph cost. Live navigation and swarm measurements are reported on pages 5-6, including failures and connectivity limitations. Offline neural accuracy and PSO convergence remain distinct from measured live behaviour. Results from one deterministic scene do not justify broad claims of robustness.')
para('<b>Remaining evaluation gaps.</b> The distributed swarm optimiser is an offline experiment, the live controller loads its stations, and the centralised/distributed comparison is a proxy. Live fault tests, repeated independent scenarios and Webots-derived neural training are not completed. These gaps may limit alignment with B2/B3; they should be disclosed or addressed before claiming full rubric completion.')
para('<b>Final submission actions.</b> Complete the original cover and declaration; verify the lecturer accepts the report and archive format; review all claims and assistance acknowledgements; preserve the final submitted copy and receipt. The package includes source code, worlds, data, figures, raw logs and run instructions. No upload or signature has been performed.')
para('<b>References (Harvard style)</b>')
para('Richfield Graduate Institute of Technology (n.d.) <i>Autonomous Systems and Robotics 700: Intelligent Autonomous Robotic Systems Project</i>. Assignment brief supplied with the module.')
para('Project experiment records (2026) <i>ASR700 offline and Webots retest logs</i>. Unpublished reproducible project outputs, 23 October. See results/tables, results/webots and README.txt. These are primary experiment records, not independent validation.')
end();c.save()
w=PdfWriter();w.add_page(PdfReader(P/'report/assignment_brief.pdf').pages[0])
for p in PdfReader(out).pages:w.add_page(p)
assert len(w.pages)==9
with (P/'report/ASR700_report.pdf').open('wb') as f:w.write(f)
print('Built 9 pages including original unsigned cover.')
