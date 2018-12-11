FROM scratch
COPY ./* .
EXPOSE 8000
CMD ["./bin/odo"]
