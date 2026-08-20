# Exhaustive reactive transition matrix — long-window video package

This package is the 49-pair reference-transition acceptance suite. Every clip uses the nominal floor and the same WBC/full controller, with a 2.5 s walking pre-roll, event A for 4.0 s, event B for 4.0 s, and a recovery tail. The purpose is to test the common continuous reference interface and handoff, not to claim that every pair is a separate physical obstacle-course experiment.

The physical obstacle claim is isolated in `go2_reactive_representatives_2026-08-20/obstacle_left_physical.mp4` and its zero-contact ground-truth evidence. Likewise, the strong physical impact and the actual floor-friction change are covered by the representative suite; the 49-pair `impact`/`low_friction` tokens here are scheduled reference events so that all directed handoffs remain comparable.

`video_manifest.json` records the actual event spans and run directories. `video_index.csv` is the quick browsing index. The sequential montage is generated only after all 49 individual clips pass recording/render checks.

`transition_metrics.csv` is the data-side audit for both event windows: reference and feedback means, orientation bounds, base height, event-active coverage, and WBC validity. `transition_qa_report.md` gives the aggregate acceptance result.
