"""The recording must not show another run's inference or crop model panels."""
import json
import threading
from pathlib import Path
from PIL import Image, ImageFont
from flight_video import FlightVideo, panel


def test_panels_are_equal_size_and_keep_the_whole_source():
    source=Image.new('RGB',(800,200),'red')
    result=panel(source)
    assert result.size==(960,540)
    assert result.getpixel((0,0))!=(255,0,0)
    assert result.getpixel((0,270))==(255,0,0)
    assert result.getpixel((959,270))==(255,0,0)


def test_another_runs_inference_is_not_counted_or_rendered(tmp_path):
    (tmp_path/'inference').mkdir()
    v=object.__new__(FlightVideo)
    v.runtime=tmp_path;v.output=tmp_path/'run';v.run_id=str(v.output)
    v.target='kim';v.title='test';v.start_time=10;v.frames=0
    v.scene_updates=v.model_updates=0;v.last_scene=v.last_model=None
    v.font=v.small=ImageFont.load_default()
    for path in (tmp_path/'live_view.jpg',tmp_path/'inference/kim.jpg'):
        Image.new('RGB',(100,100),'red').save(path)
    (tmp_path/'live_view.json').write_text(json.dumps({'run_id':v.run_id,'wall_time':11,'sim_time':2}))
    meta=tmp_path/'inference/kim.json'
    meta.write_text(json.dumps({'run_id':'a different flight','sequence':1,'wall_time':11}))
    image,trace=v._frame(12)
    assert trace['model_sequence'] is None and v.model_updates==0
    assert trace['sim_time']==2 and v.scene_updates==1
    meta.write_text(json.dumps({'run_id':v.run_id,'sequence':1,'wall_time':11}))
    image,trace=v._frame(12)
    assert trace['model_sequence']==1 and v.model_updates==1
    assert image.size==(1920,640)
