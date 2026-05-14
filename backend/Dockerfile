FROM golang:1.23-alpine AS build

WORKDIR /src
COPY go.mod ./
RUN go mod download
COPY . .
RUN go build -o /out/api-service ./cmd/api-service
RUN go build -o /out/mqtt-ingestion-service ./cmd/mqtt-ingestion-service
RUN go build -o /out/worker-service ./cmd/worker-service

FROM alpine:3.21
WORKDIR /app
COPY --from=build /out/api-service /app/api-service
COPY --from=build /out/mqtt-ingestion-service /app/mqtt-ingestion-service
COPY --from=build /out/worker-service /app/worker-service
COPY migrations /app/migrations
EXPOSE 8080
