#include "Arduino.h"
#include <WiFi.h>
#include <Wire.h>

#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"

#include "pca_action.h"

typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;

httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len){
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){
        j->len = 0;
    }
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){
        return 0;
    }
    j->len += len;
    return len;
}

static esp_err_t capture_handler(httpd_req_t *req){
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    jpg_chunking_t jchunk = {req, 0};
    
    // res = fmt2jpg_cb(gImage_medo, 50880, 106, 160, PIXFORMAT_RGB888, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
    httpd_resp_send_chunk(req, NULL, 0);
    size_t fb_len = jchunk.len;

    int64_t fr_end = esp_timer_get_time();
    Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start)/1000));
}

static esp_err_t stream_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];

    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
        } else {
            if(fb->format != PIXFORMAT_JPEG){
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                esp_camera_fb_return(fb);
                fb = NULL;
                if(!jpeg_converted){
                    Serial.println("JPEG compression failed");
                    res = ESP_FAIL;
                }
            } else {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(fb){
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if(_jpg_buf){
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();

        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        /*
        Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)"
            ,(uint32_t)(_jpg_buf_len),
            (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
            avg_frame_time, 1000.0 / avg_frame_time
        );
        */
    }

    last_frame = 0;
    return res;
}

static esp_err_t cmd_handler(httpd_req_t *req){
    char*  buf;
    size_t buf_len;
    char variable[32] = {0,};
    char value[32] = {0,};

    // Serial.println("On cmd_handler !");
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if(!buf){
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            // Serial.printf("req get buf: %s \n", buf);
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK ||
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
            } else {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        } else {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    ////////////////////////
    int res = 0;
    
    if(!strcmp(variable, "forward")) {
        Serial.println("CMD: forward !");
        quad_forward();
    } else if(!strcmp(variable, "backward")) {
        Serial.println("CMD: backward !");
        quad_backward();
    } else if(!strcmp(variable, "rotation_left")) {
        Serial.println("CMD: rotation left !");
        quad_left_rotation();
    } else if(!strcmp(variable, "rotation_right")) {
        quad_right_rotation();
        Serial.println("CMD: rotation right !");
    } else if(!strcmp(variable, "stop")) {
        Serial.println("CMD: stop !");
    } else if(!strcmp(variable, "ptz_up")) {
        uint8_t ptz_lv = quad_ptz_up();
        Serial.printf("CMD: ptz_up : %d", ptz_lv);
    } else if(!strcmp(variable, "ptz_down")) {
        uint8_t ptz_lv = quad_ptz_down();
        Serial.printf("CMD: ptz_up : %d", ptz_lv);
    } else if(!strcmp(variable, "reset")) {
        quad_reset_all();
        Serial.println("CMD: reset !");
    } else {
        res = -1;
    }

    if(res){
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);

}

static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    String page = "";
    page += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=0\">\n";
    page += "<script>var xhttp = new XMLHttpRequest();</script>";
    page += "<script>function getsend(arg) { xhttp.open('GET', 'cmd?var=' +arg  +'&' +'val=' +new Date().getTime(), true); xhttp.send() } </script>";
    page += "<p align=center><IMG SRC='http://" + WiFi.localIP().toString() + ":81/stream' style='width:300px;'></p><br/><br/>";
    // page += "<p align=center><IMG SRC='http://" + WiFi.localIP().toString() + "/capture' style='width:100px;'></p><br/><br/>";
    page += "<p align=center> <button style=width:90px;height:80px onmousedown=getsend('forward') onmouseup=getsend('stop') ontouchstart=getsend('forward') ontouchend=getsend('stop') ></button> </p>";
    page += "<p align=center>";
    page += "<button style=width:90px;height:80px onmousedown=getsend('rotation_left') onmouseup=getsend('stop') ontouchstart=getsend('rotation_left') ontouchend=getsend('stop')></button>&nbsp;";
    page += "<button style=width:90px;height:80px onmousedown=getsend('reset') onmouseup=getsend('stop')></button>&nbsp;";
    page += "<button style=width:90px;height:80px onmousedown=getsend('rotation_right') onmouseup=getsend('stop') ontouchstart=getsend('rotation_right') ontouchend=getsend('stop')></button>";
    page += "</p>";

    page += "<p align=center><button style=width:90px;height:80px onmousedown=getsend('backward') onmouseup=getsend('stop') ontouchstart=getsend('backward') ontouchend=getsend('stop') ></button></p>";  

    page += "<p align=center>";
    page += "<button style=width:90px;height:80px onmousedown=getsend('ptz_up') onmouseup=getsend('stop') ontouchstart=getsend('ptz_up') ontouchend=getsend('stop')>PTZ ^</button>&nbsp;";
    page += "<button style=width:140px;height:40px onmousedown=getsend('led')>LED</button>";
    page += "<button style=width:90px;height:80px onmousedown=getsend('ptz_down') onmouseup=getsend('stop') ontouchstart=getsend('ptz_down') ontouchend=getsend('stop')>PTZ v</button>&nbsp;";

    // page += "<button style=width:140px;height:40px onmousedown=getsend('ledon')>LED ON</button>";
    // page += "<button style=width:140px;height:40px onmousedown=getsend('ledoff')>LED OFF</button>";
    page += "</p>";


    return httpd_resp_send(req, &page[0], strlen(&page[0]));
}
void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t pic_uri = {
        .uri       = "/capture",
        .method    = HTTP_GET,
        .handler   = capture_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t cmd_uri = {
        .uri       = "/cmd",
        .method    = HTTP_GET,
        .handler   = cmd_handler,
        .user_ctx  = NULL
    };

    Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &pic_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
    }
    config.server_port += 1;
    config.ctrl_port += 1;
    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}
