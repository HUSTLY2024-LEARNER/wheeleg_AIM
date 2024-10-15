# from flask import Flask, Response, render_template, request
import argparse
# import bridge

# app = Flask(__name__)

# @app.route('/')
# def index():
#     return render_template("index.html",video_names=bridge.get_cvmat_names())

# @app.route('/video/<name>')
# def video(name):
#     return render_template("video.html", name=name)

# @app.route('/data/echarts')
# def data():
#     return render_template("echarts.html")

# @app.route('/video_feed/<name>')
# def video_feed(name):
#     return Response(bridge.get_cvmat_jpegcode(name), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--script", "-s", nargs='+', default=[], help="startup script(s) before start the web app")
    opt = parser.parse_args()
    for script in opt.script:
        print(f"running startup script: '{script}'")
        exec(open(script).read())
    #app.run(host="0.0.0.0", port=3000, threaded=True)
    while(1):
        pass