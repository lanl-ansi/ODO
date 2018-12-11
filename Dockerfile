FROM scratch
ADD . /
EXPOSE 8000
CMD ["./bin/odo"]
