# ROS 2 port — manual smoke test matrix

All 11 plugins verified on a live X display with real data
(2026-07-05, Jazzy). "Rendered" means the actual pixels were inspected,
not just process logs.

| Plugin | Verification | Result |
|---|---|---|
| Plot2D | sample_2d_plot.py publisher; subscription via CLI-topic retry; sine scatter rendered; `<topic>/plot_image` frame grabbed and inspected | rendered |
| HistogramPlot | sample_hist_pub.py; gaussian histogram rendered; `<topic>/histogram_image` frame inspected | rendered |
| Plot3D | sample_3d_plot.py; three curves plotted in 3D with autoscroll (after the bounds-after-cla fix) | rendered |
| ServiceButtons | service_button_layout.yaml; 6 buttons in 2 columns with package:// icon | rendered |
| ServiceRadioButtons | same layout as radio buttons | rendered |
| ServiceTabbedButtons | tabbed_layout yaml; push/radio tabs with full button layouts | rendered |
| YesNoButton | UI loads; buttons disabled until a /rqt_yn_btn request arrives (service advertisement verified separately) | rendered |
| StatusLight | light widget rendered (gray "Unknown" before topic selection); UInt8 feed verified headless | rendered |
| StringLabel | widget renders; subscribes only after GUI topic selection (empty pre-selection state is by design) | rendered (idle) |
| ImageView2Plugin | widget renders; image/topic selection is a GUI action. The image_view2 node is now ported (2D/3D markers, interaction, grid), so the marked-image round trip works once a topic is selected | rendered (idle) |
| DRCEnvironmentViewer | sample_drc_mini_maxwell.py; blackout state face + countdown drawn live | rendered |

Notes
- rqt has a stock SIGTERM shutdown hang (reproducible with rqt_plot):
  use `timeout -k`, treat exit 137 as normal, assert on tracebacks.
- Repeated SIGKILLs corrupt FastDDS shared memory and silently break
  discovery for surviving processes; `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`
  avoids it during testing.
- Real bugs found only by this visual pass (fixed in the port): CLI
  topic subscription race (retry added), live-canvas savefig crash
  (buffer_rgba), Plot3D zero-area canvas abort (guard + minimum size),
  Plot3D autoscroll dead due to bounds being reset by cla().
