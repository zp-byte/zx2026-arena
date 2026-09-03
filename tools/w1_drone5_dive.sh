#!/bin/bash
# drone5 FAILED 法证：从最新 ros log run 目录抽 executor_5 / nav_node_5 关键行
L=$(ls -dt ~/.ros/log/*/ | grep -v latest | head -1)
echo "run_dir=$L"
ls "$L" | head -20
echo "==== executor_5: FAILED / arrived / stuck ===="
grep -hE "FAILED|arrived|reached|stall|STALL" "$L"executor_5*.log 2>/dev/null | tail -8
echo "==== executor_5 tail 12 ===="
tail -12 "$L"executor_5*.log 2>/dev/null
echo "==== nav_node_5: goal / plan / stall tail ===="
grep -hE "new goal|plan|stall|STALL" "$L"nav_node_5*.log 2>/dev/null | tail -10
